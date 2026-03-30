package frc.robot.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ArrayBlockingQueue;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Hilo daemon que maneja una sola camara PhotonVision + su estimador de pose.
 *
 * <h2>Arquitectura de hilos</h2>
 * <p>Cada {@code CameraRunner} corre en su propio {@code Thread} daemon.
 * El hilo llama a {@link PhotonCamera#getAllUnreadResults()} cada 10 ms,
 * procesa los resultados y los deposita en una {@link ArrayBlockingQueue} thread-safe.
 * El subsistema principal drena esa cola en {@code periodic()} (hilo principal del robot).</p>
 *
 * <h2>Uso tipico</h2>
 * <pre>{@code
 * CameraRunner runner = new CameraRunner(ntInstance, "Camera_A1",
 *     VisionConstants.ROBOT_TO_CAM_A1, fieldLayout);
 * runner.start();
 *
 * // En periodic():
 * runner.drainResults(result -> {
 *     result.estimatedPose().ifPresent(est -> ...);
 * });
 * }</pre>
 */
public class CameraRunner {

    // =========================================================================
    // Resultado de una camara (pasado del hilo de vision al hilo principal)
    // =========================================================================

    /**
     * Snapshot inmutable de lo que vio una camara en un ciclo de lectura.
     *
     * @param estimatedPose  Pose estimada del robot (vacia si no hay estimacion confiable).
     * @param visibleTagIds  Lista de IDs de AprilTags visibles en el ultimo resultado.
     * @param hasTargets     {@code true} si la camara detecta al menos un target.
     * @param latestTimestamp Timestamp FPGA del resultado mas reciente [segundos].
     * @param cameraName     Nombre de la camara que genero este resultado.
     */
    public record CameraResult(
        Optional<EstimatedRobotPose> estimatedPose,
        List<Integer>                visibleTagIds,
        boolean                      hasTargets,
        double                       latestTimestamp,
        String                       cameraName
    ) {}

    // =========================================================================
    // Estado interno
    // =========================================================================

    private final String               cameraName;
    private final PhotonCamera         camera;
    private final PhotonPoseEstimator  poseEstimator;

    // Cola thread-safe entre el hilo de vision y el periodic() del subsistema.
    // Capacidad 20: evita acumulacion si periodic() se atrasa, descartando resultados viejos.
    private final ArrayBlockingQueue<CameraResult> resultQueue = new ArrayBlockingQueue<>(20);

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * Construye un CameraRunner para una camara especifica en un coprocesador.
     *
     * @param ntInstance   Instancia de NetworkTables del coprocesador (IP especifica).
     *                     Usar {@code NetworkTableInstance.create()} por separado para cada IP.
     * @param cameraName   Nombre de la camara en PhotonVision (debe coincidir exactamente).
     * @param robotToCamera Transform del centro del robot a la lente de la camara.
     * @param fieldLayout  Layout del campo con posiciones de AprilTags.
     */
    @SuppressWarnings("removal") // PhotonPoseEstimator API completa marcada para remocion en 2026.3.2
    public CameraRunner(
        NetworkTableInstance ntInstance,
        String               cameraName,
        Transform3d          robotToCamera,
        AprilTagFieldLayout  fieldLayout
    ) {
        this.cameraName = cameraName;

        // Camara conectada a la instancia NT del coprocesador especifico
        this.camera = new PhotonCamera(ntInstance, cameraName);

        // Estimador de pose: constructor 3-arg establece la estrategia principal directamente.
        // MULTI_TAG_PNP_ON_COPROCESSOR: el coprocesador resuelve PnP con todos los tags visibles.
        this.poseEstimator = new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera
        );
        // Fallback cuando solo hay un tag: usar el de menor ambiguedad.
        this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    // =========================================================================
    // Inicio del hilo
    // =========================================================================

    /**
     * Lanza el hilo daemon de esta camara.
     * Llamar una sola vez al inicializar el subsistema.
     */
    public void start() {
        Thread thread = new Thread(this::runLoop, "Vision-" + cameraName);
        thread.setDaemon(true); // No bloquea el shutdown de la JVM
        thread.start();
    }

    // =========================================================================
    // Loop del hilo de vision
    // =========================================================================

    @SuppressWarnings("BusyWait") // Polling intencional: no hay notificacion de nuevos frames
    private void runLoop() {
        while (!Thread.currentThread().isInterrupted()) {
            try {
                processAllUnreadResults();
                Thread.sleep(10); // ~100 Hz — las camaras suelen ir a 30-90 fps
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            } catch (Exception e) {
                // No dejar que una excepcion mate el hilo de vision
                // En produccion esto puede loggearse con DriverStation.reportWarning
            }
        }
    }

    private void processAllUnreadResults() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) return;

        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets()) continue;

            // Recopilar IDs de todos los tags visibles en este resultado
            List<Integer> tagIds = new ArrayList<>();
            for (PhotonTrackedTarget target : result.getTargets()) {
                int id = target.getFiducialId();
                if (id >= 0) tagIds.add(id); // -1 indica target sin ID valido
            }

            // Estimar pose con el resultado (PhotonVision aplica intrinsecas en el coprocesador)
            @SuppressWarnings("removal")
            Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);

            CameraResult cameraResult = new CameraResult(
                estimatedPose,
                List.copyOf(tagIds),      // inmutable para thread-safety
                result.hasTargets(),
                result.getTimestampSeconds(),
                cameraName
            );

            // offer() no bloquea — si la cola esta llena, descarta el resultado mas nuevo.
            // Preferimos perder frames viejos a bloquear el hilo de vision.
            resultQueue.offer(cameraResult);
        }
    }

    // =========================================================================
    // API para el subsistema (hilo principal)
    // =========================================================================

    /**
     * Drena todos los resultados pendientes en la cola y los entrega al consumidor.
     *
     * <p>Llamar desde {@code periodic()} del subsistema. Es thread-safe y no bloqueante.</p>
     *
     * @param consumer Funcion que recibe cada {@link CameraResult}.
     */
    public void drainResults(java.util.function.Consumer<CameraResult> consumer) {
        CameraResult result;
        while ((result = resultQueue.poll()) != null) {
            consumer.accept(result);
        }
    }

    /** Retorna el nombre de esta camara (para logs y SmartDashboard). */
    public String getCameraName() {
        return cameraName;
    }
}

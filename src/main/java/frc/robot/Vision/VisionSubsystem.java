package frc.robot.Vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Drive.CommandSwerveDrivetrain;

/**
 * Subsistema de vision con 4 camaras PhotonVision en 2 coprocesadores.
 *
 * <h2>Arquitectura</h2>
 * <p>Las camaras se leen directamente en {@code periodic()} — sin hilos separados.
 * PhotonVision realiza todo el procesamiento en los coprocesadores y publica los
 * resultados en el NT del RoboRIO. El robot solo deserializa y filtra.</p>
 * <ul>
 *   <li>Coprocesador derecho  (10.63.48.11): RIGHT_FRONT_CAM + RIGHT_CAM</li>
 *   <li>Coprocesador izquierdo (10.63.48.12): LEFT_FRONT_CAM + LEFT_CAM</li>
 * </ul>
 *
 * <h2>Confianza dinamica por camara</h2>
 * <p>Cada camara mantiene un valor de confianza independiente [CONFIDENCE_MIN, 1.0].
 * Sube cuando la pose es consistente con el frame anterior, baja cuando se pierden targets.
 * Los std devs se escalan inversamente con la confianza para la fusion con odometria.</p>
 *
 * <h2>SmartDashboard</h2>
 * <pre>
 * Vision/TagsVisible              — IDs de todos los tags visibles (string)
 * Vision/CAM_NAME/HasTargets      — boolean
 * Vision/CAM_NAME/TagCount        — int
 * Vision/CAM_NAME/EstX            — X estimada del robot [m]
 * Vision/CAM_NAME/EstY            — Y estimada del robot [m]
 * Vision/CAM_NAME/Timestamp       — timestamp del ultimo resultado [s]
 * Vision/CAM_NAME/Confidence      — confianza actual [CONFIDENCE_MIN, 1.0]
 * Vision/CAM_NAME/Connected       — boolean: camara conectada y recibiendo datos
 * </pre>
 */
public class VisionSubsystem extends SubsystemBase {

    // =========================================================================
    // Constantes internas
    // =========================================================================

    private static final double DIST_EXPONENT = 2.0;

    // =========================================================================
    // Estado de tags visibles (leido desde ShootCommand — mismo hilo que periodic)
    // =========================================================================

    private Set<Integer> currentVisibleTagIds = Collections.emptySet();

    // =========================================================================
    // Camaras, estimadores y estado interno
    // =========================================================================

    private final PhotonCamera[]          cameras;
    private final PhotonPoseEstimator[]   estimators;
    private final String[]                cameraNames;
    private final double[]                cameraConfidence;
    private final Pose3d[]                lastEstimatedPose;
    // Timestamp del ultimo frame ya enviado a addVisionMeasurement — evita procesar el mismo frame dos veces.
    private final double[]                lastPoseTimestamp;
    // Ciclos consecutivos sin ningun resultado — para detectar camara desconectada (~50 ciclos = 1 s).
    private final int[]                   framesSinceLastResult;

    private final CommandSwerveDrivetrain drivetrain;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * Construye el subsistema de vision e inicializa las 4 camaras PhotonVision.
     *
     * @param drivetrain Drivetrain para llamar {@code addVisionMeasurement}.
     */
    @SuppressWarnings("removal")
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        AprilTagFieldLayout fieldLayout =
            (FieldConstants.ACTIVE_FIELD_TYPE == FieldConstants.FieldType.WELDED)
                ? AprilTagFields.k2026RebuiltWelded.loadAprilTagLayoutField()
                : AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();

        // Los coprocesadores publican en el NT del RoboRIO — usar la instancia por defecto.
        NetworkTableInstance nt = NetworkTableInstance.getDefault();

        // Nombres y transforms en el mismo orden (index 0-3)
        cameraNames = new String[] {
            VisionConstants.CAM_A1_NAME,   // 0 — RIGHT_FRONT  (coproc derecho)
            VisionConstants.CAM_B1_NAME,   // 1 — RIGHT        (coproc derecho)
            VisionConstants.CAM_A2_NAME,   // 2 — LEFT_FRONT   (coproc izquierdo)
            VisionConstants.CAM_B2_NAME,   // 3 — LEFT         (coproc izquierdo)
        };

        Transform3d[] transforms = new Transform3d[] {
            VisionConstants.ROBOT_TO_CAM_A1,
            VisionConstants.ROBOT_TO_CAM_B1,
            VisionConstants.ROBOT_TO_CAM_A2,
            VisionConstants.ROBOT_TO_CAM_B2,
        };

        cameras    = new PhotonCamera[cameraNames.length];
        estimators = new PhotonPoseEstimator[cameraNames.length];

        for (int i = 0; i < cameraNames.length; i++) {
            cameras[i] = new PhotonCamera(nt, cameraNames[i]);
            estimators[i] = new PhotonPoseEstimator(
                fieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                transforms[i]
            );
            estimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }

        cameraConfidence      = new double[cameras.length];
        lastEstimatedPose     = new Pose3d[cameras.length];
        lastPoseTimestamp     = new double[cameras.length];
        framesSinceLastResult = new int[cameras.length];

        for (int i = 0; i < cameras.length; i++) {
            cameraConfidence[i]  = VisionConstants.CONFIDENCE_INITIAL;
            lastPoseTimestamp[i] = -1.0;
        }
    }

    // =========================================================================
    // periodic() — hilo principal del robot (~20 ms)
    // =========================================================================

    @Override
    @SuppressWarnings("removal")
    public void periodic() {
        Set<Integer> currentlyVisible = new HashSet<>();

        for (int i = 0; i < cameras.length; i++) {
            double estX      = 0.0;
            double estY      = 0.0;

            // getLatestResult() siempre devuelve el resultado mas reciente disponible,
            // aunque ya se haya leido antes. Esto garantiza que hasTargets y los IDs
            // visibles sean estables entre ciclos aunque no haya llegado un frame nuevo.
            PhotonPipelineResult latest = cameras[i].getLatestResult();
            boolean hasTargets          = latest.hasTargets();
            double  timestamp           = latest.getTimestampSeconds();

            // Actualizar contador de desconexion — timestamp == 0 indica sin datos aun
            if (timestamp <= 0.0) {
                framesSinceLastResult[i]++;
            } else {
                framesSinceLastResult[i] = 0;
            }

            if (hasTargets) {
                // Recopilar IDs de tags visibles (siempre desde el ultimo resultado)
                List<Integer> tagIds = new ArrayList<>();
                for (PhotonTrackedTarget target : latest.getTargets()) {
                    int id = target.getFiducialId();
                    if (id >= 0) {
                        tagIds.add(id);
                        currentlyVisible.add(id);
                    }
                }

                // Pose estimation — solo si este frame aun no fue procesado
                // (timestamp distinto al ultimo que enviamos a addVisionMeasurement)
                if (timestamp > lastPoseTimestamp[i]) {
                    lastPoseTimestamp[i] = timestamp;

                    Optional<EstimatedRobotPose> estOpt = estimators[i].update(latest);
                    if (estOpt.isPresent()) {
                        EstimatedRobotPose est = estOpt.get();
                        if (isQualityAcceptable(est, tagIds)) {
                            updateConfidence(i, est);
                            Matrix<N3, N1> stdDevs = computeStdDevs(i, est, tagIds);
                            drivetrain.addVisionMeasurement(
                                est.estimatedPose.toPose2d(),
                                est.timestampSeconds,
                                stdDevs
                            );
                            estX = est.estimatedPose.getX();
                            estY = est.estimatedPose.getY();
                        }
                    }
                }
            } else {
                decayConfidence(i);
            }

            publishCameraState(i, hasTargets, hasTargets ? latest.getTargets().size() : 0,
                               estX, estY, timestamp);
        }

        currentVisibleTagIds = Collections.unmodifiableSet(currentlyVisible);
        SmartDashboard.putString("Vision/TagsVisible", formatTagIds(currentVisibleTagIds));
    }

    // =========================================================================
    // Sistema de confianza dinamica
    // =========================================================================

    private void updateConfidence(int i, EstimatedRobotPose est) {
        Pose3d last = lastEstimatedPose[i];
        if (last != null) {
            double delta = est.estimatedPose.getTranslation()
                .getDistance(last.getTranslation());
            if (delta < VisionConstants.POSE_CONSISTENCY_THRESHOLD_METERS) {
                cameraConfidence[i] = Math.min(1.0,
                    cameraConfidence[i] + VisionConstants.CONFIDENCE_GAIN_PER_FRAME);
            } else {
                cameraConfidence[i] = Math.max(VisionConstants.CONFIDENCE_MIN,
                    cameraConfidence[i] * VisionConstants.CONFIDENCE_CONSISTENCY_PENALTY);
            }
        }
        lastEstimatedPose[i] = est.estimatedPose;
    }

    private void decayConfidence(int i) {
        cameraConfidence[i] = Math.max(VisionConstants.CONFIDENCE_MIN,
            cameraConfidence[i] * VisionConstants.CONFIDENCE_DECAY_PER_FRAME);
        lastEstimatedPose[i] = null;
    }

    // =========================================================================
    // Filtrado de calidad
    // =========================================================================

    private boolean isQualityAcceptable(EstimatedRobotPose est, List<Integer> tagIds) {
        Pose3d pose = est.estimatedPose;
        if (pose.getX() < -1.0 || pose.getX() > 17.5) return false;
        if (pose.getY() < -1.0 || pose.getY() > 9.5)  return false;

        double totalDist = 0.0;
        int    count     = 0;
        for (PhotonTrackedTarget target : est.targetsUsed) {
            totalDist += target.getBestCameraToTarget().getTranslation().getNorm();
            count++;
        }
        if (count == 0) return false;
        if (totalDist / count > VisionConstants.MAX_TAG_DISTANCE_METERS) return false;

        if (tagIds.size() == 1 && !est.targetsUsed.isEmpty()) {
            if (est.targetsUsed.get(0).getPoseAmbiguity() > VisionConstants.MAX_POSE_AMBIGUITY) {
                return false;
            }
        }
        return true;
    }

    // =========================================================================
    // Calculo de std devs
    // =========================================================================

    // Multiplicadores de std devs por camara — mismo orden que el array runners[] (indices 0-3).
    // Leen directamente de las constantes individuales para poder calibrar cada camara por separado.
    private static final double[] CAM_STD_SCALES = {
        VisionConstants.CAM_A1_STD_SCALE,   // index 0 — RIGHT_FRONT
        VisionConstants.CAM_B1_STD_SCALE,   // index 1 — RIGHT
        VisionConstants.CAM_A2_STD_SCALE,   // index 2 — LEFT_FRONT
        VisionConstants.CAM_B2_STD_SCALE,   // index 3 — LEFT
    };

    private Matrix<N3, N1> computeStdDevs(int i, EstimatedRobotPose est, List<Integer> tagIds) {
        int    nTags     = Math.max(1, tagIds.size());
        double totalDist = 0.0;
        int    count     = 0;
        for (PhotonTrackedTarget target : est.targetsUsed) {
            totalDist += target.getBestCameraToTarget().getTranslation().getNorm();
            count++;
        }
        double avgDist    = (count > 0) ? totalDist / count : VisionConstants.MAX_TAG_DISTANCE_METERS;
        double confidence = Math.max(VisionConstants.CONFIDENCE_MIN, cameraConfidence[i]);

        // Distancia normalizada a [0, 1]: 0 = junto al tag, 1 = en el limite maximo aceptado.
        // Elevada al cuadrado: penaliza fuerte las distancias grandes (comportamiento no lineal).
        double normDist = Math.min(1.0, avgDist / VisionConstants.MAX_TAG_DISTANCE_METERS);

        // Formula del factor de escala dinamico:
        //   scale = camScale * normDist² / (nTags * confidence)
        //
        //   camScale  : knob de calibracion por camara (1.0=neutro, 0.5=mas precisa, 2.0=menos)
        //   normDist² : penalizacion cuadratica por distancia — lejos = menos confianza
        //   nTags     : mas tags = mas precision → divide (reduce el factor)
        //   confidence: historial de consistencia de la camara → baja confianza = factor mayor
        //
        // scale < 1 → std devs finales MENORES que la base → MAS confianza (camara cerca, consistente)
        // scale > 1 → std devs finales MAYORES que la base → MENOS confianza (camara lejos, inconsistente)
        double rawScale = CAM_STD_SCALES[i]
            * Math.pow(normDist, DIST_EXPONENT)
            / (nTags * confidence);

        // Solo poner piso — el techo NO se limita: queremos poder desconfiar mucho si es necesario.
        double scale = Math.max(VisionConstants.CAM_STD_SCALE_FLOOR, rawScale);

        Matrix<N3, N1> base = (nTags >= 2)
            ? VisionConstants.MULTI_TAG_STD_DEVS
            : VisionConstants.SINGLE_TAG_STD_DEVS;

        return base.times(scale);
    }

    // =========================================================================
    // SmartDashboard
    // =========================================================================

    private void publishCameraState(int i, boolean hasTargets, int tagCount,
                                    double estX, double estY, double timestamp) {
        String prefix    = "Vision/" + cameraNames[i] + "/";
        // La camara se considera conectada si ha recibido datos en los ultimos ~1 segundo (50 ciclos)
        boolean connected = framesSinceLastResult[i] < 50;
        SmartDashboard.putBoolean(prefix + "HasTargets", hasTargets);
        SmartDashboard.putBoolean(prefix + "Connected",  connected);
        SmartDashboard.putNumber (prefix + "TagCount",   tagCount);
        SmartDashboard.putNumber (prefix + "EstX",       estX);
        SmartDashboard.putNumber (prefix + "EstY",       estY);
        SmartDashboard.putNumber (prefix + "Timestamp",  timestamp);
        SmartDashboard.putNumber (prefix + "Confidence", cameraConfidence[i]);
    }

    private String formatTagIds(Set<Integer> ids) {
        if (ids.isEmpty()) return "[]";
        List<Integer> sorted = new ArrayList<>(ids);
        Collections.sort(sorted);
        return sorted.toString();
    }

    // =========================================================================
    // API publica
    // =========================================================================

    /**
     * Retorna el conjunto de IDs de AprilTags visibles por cualquier camara en este ciclo.
     * Llamar desde el hilo principal (periodic, commands). Se actualiza cada ~20 ms.
     */
    public Set<Integer> getAllVisibleTagIds() {
        return currentVisibleTagIds;
    }

    /** {@code true} si al menos una camara detecta el tag con el ID dado este ciclo. */
    public boolean canSeeTag(int tagId) {
        return currentVisibleTagIds.contains(tagId);
    }

    /**
     * Retorna la confianza actual de la camara en el indice dado.
     * 0=RIGHT_FRONT, 1=RIGHT, 2=LEFT_FRONT, 3=LEFT.
     */
    public double getCameraConfidence(int camIndex) {
        if (camIndex < 0 || camIndex >= cameraConfidence.length) return 0.0;
        return cameraConfidence[camIndex];
    }
}

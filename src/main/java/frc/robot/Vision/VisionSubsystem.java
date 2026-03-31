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
import edu.wpi.first.math.VecBuilder;
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
 * <p>Las camaras se leen en {@code periodic()} — sin hilos separados.
 * PhotonVision procesa todo en los coprocesadores y publica resultados en NT.</p>
 * <ul>
 *   <li>Coprocesador derecho  (10.63.48.11): FRONT_RIGHT_CAM + RIGHT_CAM</li>
 *   <li>Coprocesador izquierdo (10.63.48.12): FRONT_LEFT_CAM  + LEFT_CAM</li>
 * </ul>
 *
 * <h2>Algoritmo de std devs — stateless</h2>
 * <p>Los std devs se calculan directamente de la geometria de cada medicion,
 * sin ningun acumulador de historial. Mas tags y menor distancia = menor std dev
 * (mas confianza). Con 1 solo tag, el heading se ignora (std dev = 9999).</p>
 *
 * <h2>SmartDashboard</h2>
 * <pre>
 * Vision/TagsVisible              — IDs de todos los tags visibles
 * Vision/CAM_NAME/HasTargets      — boolean
 * Vision/CAM_NAME/TagCount        — int
 * Vision/CAM_NAME/EstX            — X estimada del robot [m]
 * Vision/CAM_NAME/EstY            — Y estimada del robot [m]
 * Vision/CAM_NAME/Timestamp       — timestamp del ultimo resultado [s]
 * Vision/CAM_NAME/XyStd           — std dev X/Y calculado [m] (0 = no hay medicion)
 * Vision/CAM_NAME/Connected       — boolean: camara recibiendo datos
 * </pre>
 */
public class VisionSubsystem extends SubsystemBase {

    // Std devs fijos por camara [xy_m, theta_rad] — mismo orden que cameras[]
    private static final double[] CAM_STD_XY = {
        VisionConstants.CAM_A1_STD_XY,   // 0 — FRONT_RIGHT
        VisionConstants.CAM_B1_STD_XY,   // 1 — RIGHT
        VisionConstants.CAM_A2_STD_XY,   // 2 — FRONT_LEFT
        VisionConstants.CAM_B2_STD_XY,   // 3 — LEFT
    };
    private static final double[] CAM_STD_THETA = {
        VisionConstants.CAM_A1_STD_THETA,
        VisionConstants.CAM_B1_STD_THETA,
        VisionConstants.CAM_A2_STD_THETA,
        VisionConstants.CAM_B2_STD_THETA,
    };

    // =========================================================================
    // Tags visibles este ciclo (leido desde otros comandos — mismo hilo)
    // =========================================================================

    private Set<Integer> currentVisibleTagIds = Collections.emptySet();

    // =========================================================================
    // Camaras, estimadores y estado minimo
    // =========================================================================

    private final PhotonCamera[]        cameras;
    private final PhotonPoseEstimator[] estimators;
    private final String[]              cameraNames;

    // Timestamp del ultimo frame ya enviado — evita procesar el mismo frame dos veces.
    private final double[]              lastPoseTimestamp;

    // Ciclos consecutivos sin datos — detecta camara desconectada (~50 ciclos = 1 s).
    private final int[]                 framesSinceLastResult;

    private final CommandSwerveDrivetrain drivetrain;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * @param drivetrain Drivetrain para llamar {@code addVisionMeasurement}.
     */
    @SuppressWarnings("removal")
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        AprilTagFieldLayout fieldLayout =
            (FieldConstants.ACTIVE_FIELD_TYPE == FieldConstants.FieldType.WELDED)
                ? AprilTagFields.k2026RebuiltWelded.loadAprilTagLayoutField()
                : AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();

        NetworkTableInstance nt = NetworkTableInstance.getDefault();

        cameraNames = new String[] {
            VisionConstants.CAM_A1_NAME,   // 0 — FRONT_RIGHT
            VisionConstants.CAM_B1_NAME,   // 1 — RIGHT
            VisionConstants.CAM_A2_NAME,   // 2 — FRONT_LEFT
            VisionConstants.CAM_B2_NAME,   // 3 — LEFT
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

        lastPoseTimestamp     = new double[cameras.length];
        framesSinceLastResult = new int[cameras.length];

        for (int i = 0; i < cameras.length; i++) {
            lastPoseTimestamp[i] = -1.0;
        }
    }

    // =========================================================================
    // periodic() — hilo principal (~20 ms)
    // =========================================================================

    @Override
    @SuppressWarnings("removal")
    public void periodic() {
        Set<Integer> currentlyVisible = new HashSet<>();

        for (int i = 0; i < cameras.length; i++) {
            double estX = 0.0, estY = 0.0, xyStd = 0.0;

            PhotonPipelineResult latest    = cameras[i].getLatestResult();
            boolean              hasTargets = latest.hasTargets();
            double               timestamp  = latest.getTimestampSeconds();

            // Detectar desconexion: timestamp <= 0 significa que no hay datos aun
            if (timestamp <= 0.0) {
                framesSinceLastResult[i]++;
            } else {
                framesSinceLastResult[i] = 0;
            }

            if (hasTargets) {
                // Recolectar IDs de tags visibles en este ciclo
                List<Integer> tagIds = new ArrayList<>();
                for (PhotonTrackedTarget t : latest.getTargets()) {
                    int id = t.getFiducialId();
                    if (id >= 0) {
                        tagIds.add(id);
                        currentlyVisible.add(id);
                    }
                }

                // Procesar solo si es un frame nuevo (timestamp distinto al anterior)
                if (timestamp > lastPoseTimestamp[i]) {
                    lastPoseTimestamp[i] = timestamp;

                    Optional<EstimatedRobotPose> estOpt = estimators[i].update(latest);
                    if (estOpt.isPresent()) {
                        EstimatedRobotPose est = estOpt.get();

                        if (isQualityAcceptable(est, tagIds)) {
                            // Gate de velocidad: no corregir mientras el robot se mueve rapido.
                            // A alta velocidad vision es menos precisa y sus correcciones
                            // oscilan contra la odometria generando distancia fantasma.
                            var speeds = drivetrain.getState().Speeds;
                            double robotSpeed = Math.hypot(
                                speeds.vxMetersPerSecond,
                                speeds.vyMetersPerSecond
                            );

                            // Rechazo de outliers: pose muy lejana a la odometria = glitch.
                            double poseJump = est.estimatedPose.toPose2d().getTranslation()
                                .getDistance(drivetrain.getState().Pose.getTranslation());

                            if (robotSpeed <= VisionConstants.VISION_MAX_SPEED_MPS
                                    && poseJump <= VisionConstants.MAX_VISION_POSE_JUMP_METERS) {
                                Matrix<N3, N1> stdDevs = computeStdDevs(i);
                                xyStd = stdDevs.get(0, 0);
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
                }
            }

            publishCameraState(i, hasTargets, hasTargets ? latest.getTargets().size() : 0,
                               estX, estY, timestamp, xyStd);
        }

        currentVisibleTagIds = Collections.unmodifiableSet(currentlyVisible);
        SmartDashboard.putString("Vision/TagsVisible", formatTagIds(currentVisibleTagIds));
    }

    // =========================================================================
    // Std devs fijos por camara
    // =========================================================================

    private Matrix<N3, N1> computeStdDevs(int camIdx) {
        return VecBuilder.fill(CAM_STD_XY[camIdx], CAM_STD_XY[camIdx], CAM_STD_THETA[camIdx]);
    }

    // =========================================================================
    // Filtrado de calidad de la estimacion
    // =========================================================================

    private boolean isQualityAcceptable(EstimatedRobotPose est, List<Integer> tagIds) {
        // Rechazar poses fuera del campo
        double x = est.estimatedPose.getX();
        double y = est.estimatedPose.getY();
        if (x < -1.0 || x > 17.5 || y < -1.0 || y > 9.5) return false;

        // Rechazar si la distancia promedio a los tags es demasiado grande
        double totalDist = 0.0;
        int    count     = 0;
        for (PhotonTrackedTarget t : est.targetsUsed) {
            totalDist += t.getBestCameraToTarget().getTranslation().getNorm();
            count++;
        }
        if (count == 0 || totalDist / count > VisionConstants.MAX_TAG_DISTANCE_METERS) return false;

        // Rechazar single-tag con ambiguedad alta
        if (tagIds.size() == 1 && !est.targetsUsed.isEmpty()) {
            if (est.targetsUsed.get(0).getPoseAmbiguity() > VisionConstants.MAX_POSE_AMBIGUITY) {
                return false;
            }
        }

        return true;
    }

    // =========================================================================
    // SmartDashboard
    // =========================================================================

    private void publishCameraState(int i, boolean hasTargets, int tagCount,
                                    double estX, double estY, double timestamp,
                                    double xyStd) {
        String  prefix    = "Vision/" + cameraNames[i] + "/";
        boolean connected = framesSinceLastResult[i] < 50;
        // Confidence: 1.0 = medicion aceptada y enviada al filtro, 0.0 = sin medicion valida.
        double confidence = (xyStd > 0.0) ? 1.0 : 0.0;
        SmartDashboard.putBoolean(prefix + "HasTargets",  hasTargets);
        SmartDashboard.putBoolean(prefix + "Connected",   connected);
        SmartDashboard.putNumber (prefix + "TagCount",    tagCount);
        SmartDashboard.putNumber (prefix + "EstX",        estX);
        SmartDashboard.putNumber (prefix + "EstY",        estY);
        SmartDashboard.putNumber (prefix + "Timestamp",   timestamp);
        SmartDashboard.putNumber (prefix + "Confidence",  confidence);
        SmartDashboard.putNumber (prefix + "XyStd",       xyStd);
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

    /** IDs de todos los AprilTags visibles por cualquier camara este ciclo. */
    public Set<Integer> getAllVisibleTagIds() {
        return currentVisibleTagIds;
    }

    /** {@code true} si al menos una camara detecta el tag con el ID dado este ciclo. */
    public boolean canSeeTag(int tagId) {
        return currentVisibleTagIds.contains(tagId);
    }
}

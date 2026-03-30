package frc.robot.Drive;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drive.TunerConstants.TunerSwerveDrivetrain;

/**
 * Subsistema del swerve drivetrain basado en Phoenix 6.
 *
 * Extiende TunerSwerveDrivetrain (generado por Tuner X) e implementa Subsystem
 * para integrarse con el sistema de comandos de WPILib.
 *
 * Incluye:
 * - Control field-centric y robot-centric
 * - Integracion completa con PathPlanner para autonomo
 * - Rutinas SysId para caracterizar los motores
 * - Hilo de simulacion para pruebas sin hardware
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    // Periodo del hilo de simulacion — referenciado desde DriveConstants para un solo punto de verdad
    private static final double SIM_LOOP_PERIOD_SECONDS = DriveConstants.SIM_LOOP_PERIOD_SECONDS;
    private Notifier simNotifier = null;
    private double lastSimTime;

    // Perspectiva del operador segun alianza
    private static final Rotation2d BLUE_ALLIANCE_FORWARD = Rotation2d.kZero;
    private static final Rotation2d RED_ALLIANCE_FORWARD  = Rotation2d.k180deg;
    private boolean hasAppliedOperatorPerspective = false;

    // Control request reutilizable para trayectorias de PathPlanner.
    // Se crea una sola vez para evitar generar objetos nuevos cada ciclo.
    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    // Requests para caracterizacion SysId
    private final SwerveRequest.SysIdSwerveTranslation sysIdTranslationRequest = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains  sysIdSteerRequest       = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation    sysIdRotationRequest    = new SwerveRequest.SysIdSwerveRotation();

    // Rutina SysId para translacion — encuentra ganancias PID de los motores de drive
    private final SysIdRoutine sysIdTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(DriveConstants.SYSID_TRANSLATION_STEP_VOLTS),
            null,
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(sysIdTranslationRequest.withVolts(output)),
            null,
            this
        )
    );

    // Rutina SysId para steer — encuentra ganancias PID de los motores de giro
    @SuppressWarnings("unused")
    private final SysIdRoutine sysIdSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(DriveConstants.SYSID_STEER_STEP_VOLTS),
            null,
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(sysIdSteerRequest.withVolts(volts)),
            null,
            this
        )
    );

    // Rutina SysId para rotacion — encuentra ganancias del heading controller
    @SuppressWarnings("unused")
    private final SysIdRoutine sysIdRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(DriveConstants.SYSID_ROTATION_RAMP_RATE_VOLTS_PER_S).per(Second),
            Volts.of(DriveConstants.SYSID_ROTATION_STEP_VOLTS),
            null,
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(sysIdRotationRequest.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    // Rutina activa para SysId (se puede cambiar segun lo que se quiera caracterizar)
    private final SysIdRoutine activeSysIdRoutine = sysIdTranslation;

    // -------------------------------------------------------------------------
    // Constructores
    // -------------------------------------------------------------------------

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    // -------------------------------------------------------------------------
    // PathPlanner
    // -------------------------------------------------------------------------

    /**
     * Configura PathPlanner AutoBuilder para que pueda controlar el drivetrain
     * durante el autonomo.
     *
     * Usa feedforwards de rueda para mejor seguimiento de trayectorias.
     * RobotConfig se lee del archivo generado por PathPlanner GUI en deploy/pathplanner/settings.json.
     * Si ese archivo no existe todavia, el error se reporta al DriverStation sin crashear el robot.
     */
    private void configurePathPlanner() {
        try {
            AutoBuilder.configure(
                // Proveedor de pose actual del robot
                () -> getState().Pose,

                // Resetear la odometria a una pose dada (PathPlanner lo llama al inicio del auto)
                this::resetPose,

                // Velocidades actuales del robot relativas al robot
                () -> getState().Speeds,

                // Aplicar velocidades calculadas por PathPlanner con feedforwards de rueda
                (speeds, feedforwards) -> setControl(
                    autoRequest
                        .withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),

                // Controlador holonomico con PIDs de DriveConstants
                new PPHolonomicDriveController(
                    new PIDConstants(
                        DriveConstants.PATHPLANNER_TRANSLATION_P,
                        DriveConstants.PATHPLANNER_TRANSLATION_I,
                        DriveConstants.PATHPLANNER_TRANSLATION_D
                    ),
                    new PIDConstants(
                        DriveConstants.PATHPLANNER_ROTATION_P,
                        DriveConstants.PATHPLANNER_ROTATION_I,
                        DriveConstants.PATHPLANNER_ROTATION_D
                    )
                ),

                // Configuracion del robot (masa, inercia, modulos) leida desde PathPlanner GUI
                RobotConfig.fromGUISettings(),

                // Voltear la trayectoria si estamos en alianza roja
                () -> DriverStation.getAlliance()
                    .map(alliance -> alliance == Alliance.Red)
                    .orElse(false),

                // Este subsistema es requerido por los comandos de PathPlanner
                this
            );
        } catch (java.io.IOException | RuntimeException e) {
            // IOException: no se pudo leer deploy/pathplanner/settings.json
            // RuntimeException: error de configuracion del RobotConfig
            DriverStation.reportError(
                "[PathPlanner] Error al configurar AutoBuilder: " + e.getMessage(),
                e.getStackTrace()
            );
        } catch (Exception e) {
            // Cualquier otro error inesperado (ej. ParseException del JSON)
            DriverStation.reportError(
                "[PathPlanner] Error inesperado en AutoBuilder: " + e.getMessage(),
                e.getStackTrace()
            );
        }
    }

    // -------------------------------------------------------------------------
    // Metodos de control
    // -------------------------------------------------------------------------

    /**
     * Retorna un comando que aplica el request de swerve dado mientras corre.
     *
     * @param request Funcion que retorna el SwerveRequest a aplicar
     * @return Comando que aplica el request continuamente
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    // -------------------------------------------------------------------------
    // SysId
    // -------------------------------------------------------------------------

    /**
     * Corre la rutina quasistatic SysId activa en la direccion especificada.
     * Nota: Cada rutina debe correrse exactamente una vez por log.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return activeSysIdRoutine.quasistatic(direction);
    }

    /**
     * Corre la rutina dynamic SysId activa en la direccion especificada.
     * Nota: Cada rutina debe correrse exactamente una vez por log.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return activeSysIdRoutine.dynamic(direction);
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        // Aplica la perspectiva del operador segun la alianza.
        // Se aplica al inicio y cuando el robot esta deshabilitado para
        // que el driver siempre vea "adelante" como la direccion correcta.
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? RED_ALLIANCE_FORWARD
                        : BLUE_ALLIANCE_FORWARD
                );
                hasAppliedOperatorPerspective = true;
            });
        }
    }

    // -------------------------------------------------------------------------
    // Vision
    // -------------------------------------------------------------------------

    /**
     * Agrega una medicion de vision al filtro de Kalman para corregir la odometria.
     *
     * @param visionRobotPoseMeters Pose del robot estimada por la camara
     * @param timestampSeconds      Timestamp de la medicion en segundos (FPGA time)
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Agrega una medicion de vision con desviaciones estandar personalizadas.
     *
     * @param visionRobotPoseMeters    Pose del robot estimada por la camara
     * @param timestampSeconds         Timestamp de la medicion en segundos
     * @param visionMeasurementStdDevs Desviaciones estandar [x, y, theta] en metros y radianes
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(
            visionRobotPoseMeters,
            Utils.fpgaToCurrentTime(timestampSeconds),
            visionMeasurementStdDevs
        );
    }

    /**
     * Retorna la pose del robot en un timestamp dado (si el buffer no esta vacio).
     *
     * @param timestampSeconds Timestamp en segundos
     * @return Pose en ese momento, o Optional.empty() si el buffer esta vacio
     */
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    // -------------------------------------------------------------------------
    // Simulacion
    // -------------------------------------------------------------------------

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        // Corre la simulacion a 250 Hz para que los PIDs se comporten correctamente
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(SIM_LOOP_PERIOD_SECONDS);
    }
}

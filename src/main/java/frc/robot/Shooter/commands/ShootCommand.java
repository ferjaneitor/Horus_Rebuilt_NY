package frc.robot.Shooter.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Drive.CommandSwerveDrivetrain;
import frc.robot.Intake.IntakePivotSubsystem;
import frc.robot.Shooter.ShooterMath;
import frc.robot.Shooter.ShooterMath.ShooterMode;
import frc.robot.Shooter.ShooterMath.ShooterResult;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Vision.VisionSubsystem;

/**
 * Comando de disparo del robot Horus.
 *
 * <h2>Modos de operacion</h2>
 * <p>El modo activo se lee del {@code SendableChooser} en el SmartDashboard al inicio
 * de cada disparo ({@code initialize()}) y no cambia durante el disparo.</p>
 *
 * <ul>
 *   <li><b>VARIABLE_HOOD</b> (defecto) — ShooterMath calcula angulo + RPM segun distancia.
 *       Vision tracking activo, hood se mueve, auto-aim heading disponible.</li>
 *   <li><b>FIXED_HOOD</b> — Hood permanece en posicion home. Solo el flywheel ajusta RPM
 *       con lookup table segun distancia. Sin vision tracking ni auto-aim.</li>
 * </ul>
 *
 * <h2>Flujo comun a ambos modos</h2>
 * <ol>
 *   <li>Guarda setpoint del pivot del intake.</li>
 *   <li>Aplica parametros iniciales segun modo.</li>
 *   <li>Cuando {@link ShooterSubsystem#isReadyToShoot()} es true, activa el indexer.</li>
 *   <li>Mientras el indexer corre, el pivot se retrae progresivamente (ratchet).</li>
 *   <li>Al soltar: detiene indexer, coast flywheel, hood a home, restaura pivot.</li>
 * </ol>
 *
 * <h2>Uso en RobotContainer</h2>
 * <pre>{@code
 * driverController.rightTrigger().whileTrue(
 *     new ShootCommand(shooter, vision, drivetrain, intakePivot, shooterModeChooser::getSelected)
 * );
 * }</pre>
 */
public class ShootCommand extends Command {

    // =========================================================================
    // Dependencias
    // =========================================================================

    private final ShooterSubsystem              shooter;
    private final VisionSubsystem               vision;
    private final CommandSwerveDrivetrain        drivetrain;
    private final IntakePivotSubsystem          intakePivot;
    private final Supplier<ShooterMode>         shooterModeSupplier;

    // =========================================================================
    // Estado del comando
    // =========================================================================

    /** Modo activo para este disparo — se fija en initialize(). */
    private ShooterMode activeMode;

    /** true = modo vision (tag-based), solo relevante en VARIABLE_HOOD. */
    private boolean useVisionMode;

    /** Setpoint del pivot antes de disparar — se restaura en end(). */
    private double pivotSetpointBeforeShoot;

    // =========================================================================
    // Estado del ratchet progresivo
    // =========================================================================

    private double  ratchetSetpoint;
    private boolean ratchetPhaseIsRetracting;
    private boolean ratchetStarted;
    private final Timer ratchetTimer = new Timer();

    // =========================================================================
    // Latch del indexer
    // =========================================================================

    /**
     * true desde el primer ciclo en que el flywheel llega a velocidad objetivo.
     * Una vez activado, el indexer corre continuamente hasta end() sin importar
     * fluctuaciones de RPM causadas por el paso de la pelota.
     * Se resetea en initialize() para cada nuevo disparo.
     */
    private boolean indexerLatch = false;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * @param shooter             Subsistema del shooter (requerido).
     * @param vision              Subsistema de vision (lectura, no requerido).
     * @param drivetrain          Drivetrain para pose y velocidades (no requerido).
     * @param intakePivot         Pivot del intake (requerido — retraccion progresiva).
     * @param shooterModeSupplier Proveedor del modo activo (ej. {@code chooser::getSelected}).
     */
    public ShootCommand(
        ShooterSubsystem        shooter,
        VisionSubsystem         vision,
        CommandSwerveDrivetrain drivetrain,
        IntakePivotSubsystem    intakePivot,
        Supplier<ShooterMode>   shooterModeSupplier
    ) {
        this.shooter             = shooter;
        this.vision              = vision;
        this.drivetrain          = drivetrain;
        this.intakePivot         = intakePivot;
        this.shooterModeSupplier = shooterModeSupplier;
        addRequirements(shooter, intakePivot);
    }

    // =========================================================================
    // initialize()
    // =========================================================================

    @Override
    public void initialize() {
        activeMode = shooterModeSupplier.get();

        pivotSetpointBeforeShoot = intakePivot.getCurrentSetpoint();
        ratchetSetpoint          = pivotSetpointBeforeShoot;
        ratchetStarted           = false;
        ratchetPhaseIsRetracting = true;
        indexerLatch             = false;

        if (activeMode == ShooterMode.FIXED_HOOD) {
            initFixedHood();
        } else {
            initVariableHood();
        }
    }

    // =========================================================================
    // execute()
    // =========================================================================

    @Override
    public void execute() {
        // FIXED_HOOD: el hood no se mueve — solo esperar que el flywheel llegue a velocidad.
        // VARIABLE_HOOD: esperar flywheel + angulo del hood.
        boolean ready = (activeMode == ShooterMode.FIXED_HOOD)
            ? shooter.isAtTargetRPM()
            : shooter.isReadyToShoot();

        // Latch: una vez que el flywheel llega a velocidad por primera vez, el indexer
        // se mantiene activo aunque las RPM fluctúen al pasar pelotas.
        if (ready && !indexerLatch) {
            indexerLatch = true;
            startRatchetIfNeeded();
        }
        if (indexerLatch) {
            shooter.runIndexer();
        }
        // Nunca llamar stopIndexer() aquí — solo en end().

        updateProgressiveRetract();

        if (activeMode == ShooterMode.FIXED_HOOD) {
            executeFixedHood();
        } else {
            executeVariableHood();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // =========================================================================
    // end()
    // =========================================================================

    @Override
    public void end(boolean interrupted) {
        indexerLatch = false;
        shooter.stopIndexer();
        shooter.coastFlywheel();
        shooter.homeHood();
        intakePivot.setSetpoint(pivotSetpointBeforeShoot);
    }

    // =========================================================================
    // Logica de modo VARIABLE_HOOD
    // =========================================================================

    private void initVariableHood() {
        useVisionMode = false;

        if (!vision.getAllVisibleTagIds().isEmpty()) {
            ShooterResult result = calculateFromVision();
            if (result != null && result.isValidShot()) {
                useVisionMode = true;
                applyVariableResult(result);
                return;
            }
        }

        applyDefaultShot();
    }

    private void executeVariableHood() {
        if (!useVisionMode) return;
        ShooterResult freshResult = calculateFromVision();
        if (freshResult != null && freshResult.isValidShot()) {
            applyVariableResult(freshResult);
        }
        // Si se pierden los tags, se mantienen los ultimos parametros aplicados.
    }

    private ShooterResult calculateFromVision() {
        if (vision.getAllVisibleTagIds().isEmpty()) return null;
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return ShooterMath.calculateAutoTarget(
            drivetrain.getState().Pose,
            drivetrain.getState().Speeds,
            alliance
        );
    }

    private void applyVariableResult(ShooterResult result) {
        shooter.setFlywheelRPM(result.flywheelRPM());
        shooter.setHoodAngle(result.shooterAngleDegrees());
        ShooterMath.publishToSmartDashboard(result);
    }

    private void applyDefaultShot() {
        useVisionMode = false;
        shooter.setFlywheelRPM(ShooterConstants.DEFAULT_SHOT_RPM);
        shooter.homeHood();
    }

    // =========================================================================
    // Logica de modo FIXED_HOOD
    // =========================================================================

    private void initFixedHood() {
        shooter.homeHood(); // Hood permanece fijo en home durante todo el disparo

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Pose2d pose = drivetrain.getState().Pose;
        if (pose == null) {
            // Sin pose — usar RPM de fallback hasta que la odometria este disponible
            shooter.setFlywheelRPM(ShooterConstants.FIXED_HOOD_DEFAULT_RPM);
            return;
        }
        ShooterResult result = ShooterMath.calculateFixedHood(pose, alliance);
        ShooterMath.publishToSmartDashboard(result);
        shooter.setFlywheelRPM(result.isValidShot()
            ? result.flywheelRPM()
            : ShooterConstants.FIXED_HOOD_DEFAULT_RPM);
    }

    private void executeFixedHood() {
        // Una vez que el indexer está latched, el disparo está en curso.
        // No cambiar el target RPM: si la pose se corrige por visión a mitad del ciclo,
        // el nuevo RPM calcualdo puede diferir 1000+ RPM del actual y matar el disparo.
        if (indexerLatch) return;

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Pose2d pose = drivetrain.getState().Pose;
        if (pose == null) return;
        ShooterResult result = ShooterMath.calculateFixedHood(pose, alliance);
        ShooterMath.publishToSmartDashboard(result);
        if (result.isValidShot()) {
            shooter.setFlywheelRPM(result.flywheelRPM());
        }
    }

    // =========================================================================
    // Retraccion progresiva del pivot (ratchet) — comun a ambos modos
    // =========================================================================

    private void startRatchetIfNeeded() {
        if (ratchetStarted) return;

        ratchetStarted           = true;
        ratchetPhaseIsRetracting = true;
        ratchetSetpoint = Math.max(
            IntakeConstants.PIVOT_RETRACTED_POSITION_ROTATIONS,
            pivotSetpointBeforeShoot - IntakeConstants.PIVOT_RATCHET_STEP_UP_ROTATIONS
        );
        intakePivot.setSetpoint(ratchetSetpoint);
        ratchetTimer.restart();
    }

    private void updateProgressiveRetract() {
        if (!ratchetStarted) return;
        if (ratchetSetpoint <= IntakeConstants.PIVOT_RETRACTED_POSITION_ROTATIONS) return;

        if (ratchetPhaseIsRetracting
                && ratchetTimer.hasElapsed(IntakeConstants.PIVOT_RATCHET_UP_SECONDS)) {
            ratchetPhaseIsRetracting = false;
            ratchetSetpoint = Math.min(
                pivotSetpointBeforeShoot,
                ratchetSetpoint + IntakeConstants.PIVOT_RATCHET_STEP_DOWN_ROTATIONS
            );
            intakePivot.setSetpoint(ratchetSetpoint);
            ratchetTimer.restart();
            return;
        }

        if (!ratchetPhaseIsRetracting
                && ratchetTimer.hasElapsed(IntakeConstants.PIVOT_RATCHET_DOWN_SECONDS)) {
            ratchetPhaseIsRetracting = true;
            ratchetSetpoint = Math.max(
                IntakeConstants.PIVOT_RETRACTED_POSITION_ROTATIONS,
                ratchetSetpoint - IntakeConstants.PIVOT_RATCHET_STEP_UP_ROTATIONS
            );
            intakePivot.setSetpoint(ratchetSetpoint);
            ratchetTimer.restart();
        }
    }
}

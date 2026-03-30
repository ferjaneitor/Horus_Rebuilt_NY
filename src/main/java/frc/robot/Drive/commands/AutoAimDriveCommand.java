package frc.robot.Drive.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConstants;
import frc.robot.Drive.CommandSwerveDrivetrain;
import frc.robot.Shooter.ShooterMath;
import frc.robot.Shooter.ShooterMath.ShooterResult;

/**
 * Comando de manejo con auto-aim: el robot apunta automaticamente hacia el objetivo
 * calculado por {@link ShooterMath}, mientras el driver controla la traslacion XY.
 *
 * <h2>Comportamiento</h2>
 * <ul>
 *   <li><b>Omega</b> — controlada automaticamente via {@link SwerveRequest.FieldCentricFacingAngle}.
 *       El PID interno de CTRE gira el robot hasta que apunte al objetivo.</li>
 *   <li><b>Traslacion XY</b> — controlada por el driver con los suppliers dados
 *       (tipicamente joystick izquierdo).</li>
 *   <li><b>Objetivo</b> — calculado por {@link ShooterMath#calculateAutoTarget} en cada ciclo:
 *       SCORING en zona propia, PASSING (mas cercano en Y) en zona neutral/enemiga.</li>
 *   <li><b>Sin tiro valido</b> — si {@code calculateAutoTarget} retorna invalido,
 *       el robot mantiene su heading actual (sin giro forzado).</li>
 * </ul>
 *
 * <h2>SmartDashboard</h2>
 * <pre>
 * AutoAim/TargetHeading  — heading objetivo del robot [grados, -180 a 180]
 * AutoAim/CurrentHeading — heading actual del robot [grados]
 * AutoAim/Aligned        — true si el error de heading < AUTO_AIM_TOLERANCE_DEGREES
 * AutoAim/Zone           — zona detectada (RED_ALLIANCE / NEUTRAL / BLUE_ALLIANCE)
 * AutoAim/ValidTarget    — true si ShooterMath calculo un tiro valido
 * </pre>
 *
 * <h2>Uso en RobotContainer</h2>
 * <pre>{@code
 * private final AutoAimDriveCommand autoAimDrive = new AutoAimDriveCommand(
 *     drivetrain,
 *     () -> -driverController.getLeftY()  * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
 *     () -> -driverController.getLeftX()  * DriveConstants.MAX_SPEED_METERS_PER_SECOND
 * );
 *
 * // X: auto-aim mientras se mantiene presionado
 * driverController.x().whileTrue(autoAimDrive);
 * }</pre>
 */
public class AutoAimDriveCommand extends Command {

    // =========================================================================
    // Dependencias
    // =========================================================================

    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier          translationXSupplier; // [m/s] adelante/atras
    private final DoubleSupplier          translationYSupplier; // [m/s] izquierda/derecha

    // =========================================================================
    // Request de swerve
    // =========================================================================

    // FieldCentricFacingAngle: controla X/Y del driver + omega automatico para apuntar
    private final SwerveRequest.FieldCentricFacingAngle aimRequest;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * @param drivetrain         Subsistema del drivetrain (requerido por este comando).
     * @param translationXSupplier Velocidad en X del campo [m/s] — tipicamente -leftY * maxSpeed.
     * @param translationYSupplier Velocidad en Y del campo [m/s] — tipicamente -leftX * maxSpeed.
     */
    public AutoAimDriveCommand(
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier          translationXSupplier,
        DoubleSupplier          translationYSupplier
    ) {
        this.drivetrain           = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;

        // Configurar el request de auto-aim
        this.aimRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(DriveConstants.MAX_SPEED_METERS_PER_SECOND
                          * DriveConstants.JOYSTICK_DEADBAND_PERCENT)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // PID del heading controller interno de CTRE Phoenix 6
        // kP: proporcional, kI: integral, kD: derivativo
        this.aimRequest.HeadingController.setPID(
            DriveConstants.AUTO_AIM_P,
            DriveConstants.AUTO_AIM_I,
            DriveConstants.AUTO_AIM_D
        );

        // Solo el drivetrain es requerido — el auto-aim no toca el shooter
        addRequirements(drivetrain);
    }

    // =========================================================================
    // execute() — cada ~20 ms
    // =========================================================================

    @Override
    public void execute() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        // Calcular heading objetivo segun zona y alianza
        ShooterResult result = ShooterMath.calculateAutoTarget(
            drivetrain.getState().Pose,
            drivetrain.getState().Speeds,
            alliance
        );

        // Si el tiro es valido, apuntar al heading calculado.
        // Si no (demasiado lejos, angulo fuera de rango, etc.), mantener heading actual
        // para que el robot no gire aleatoriamente.
        Rotation2d targetHeading;
        boolean    validTarget;

        if (result != null && result.isValidShot()) {
            targetHeading = Rotation2d.fromDegrees(result.robotHeadingDegrees());
            validTarget   = true;
        } else {
            targetHeading = drivetrain.getState().Pose.getRotation();
            validTarget   = false;
        }

        // Aplicar request: XY del driver + heading automatico
        drivetrain.setControl(
            aimRequest
                .withVelocityX(translationXSupplier.getAsDouble())
                .withVelocityY(translationYSupplier.getAsDouble())
                .withTargetDirection(targetHeading)
        );

        // --- Telemetria ---
        double currentHeadingDeg = drivetrain.getState().Pose.getRotation().getDegrees();
        double targetHeadingDeg  = targetHeading.getDegrees();
        double headingError      = Math.abs(currentHeadingDeg - targetHeadingDeg);
        // Normalizar error a [-180, 180]
        if (headingError > 180.0) headingError = 360.0 - headingError;

        SmartDashboard.putNumber ("AutoAim/TargetHeading",  targetHeadingDeg);
        SmartDashboard.putNumber ("AutoAim/CurrentHeading", currentHeadingDeg);
        SmartDashboard.putBoolean("AutoAim/Aligned",
            headingError < DriveConstants.AUTO_AIM_TOLERANCE_DEGREES && validTarget);
        SmartDashboard.putString ("AutoAim/Zone",
            ShooterMath.detectZone(drivetrain.getState().Pose).name());
        SmartDashboard.putBoolean("AutoAim/ValidTarget", validTarget);
    }

    // =========================================================================
    // isFinished / end
    // =========================================================================

    @Override
    public boolean isFinished() {
        return false; // Corre hasta que se suelta el boton X
    }

    @Override
    public void end(boolean interrupted) {
        // No se necesita cleanup:
        // Al terminar este comando, el default command del drivetrain retoma el control
        // y el robot vuelve al manejo manual completo (incluido omega).
        SmartDashboard.putBoolean("AutoAim/Aligned",      false);
        SmartDashboard.putBoolean("AutoAim/ValidTarget",  false);
    }
}

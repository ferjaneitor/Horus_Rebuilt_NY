package frc.robot.Util.Manual_Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.ShooterSubsystem;

/**
 * Comando para controlar el flywheel izquierdo de manera manual.
 *
 * <p>Diseñado para caracterizacion y diagnostico. La salida la provee un
 * {@link DoubleSupplier} (tipicamente un eje del joystick o un valor de
 * SmartDashboard) y se clampea al maximo configurado en
 * {@code ShooterConstants.FLYWHEEL_MANUAL_MAX_OUTPUT}.</p>
 *
 * <p>Uso tipico en RobotContainer (modo de desarrollo):</p>
 * <pre>
 *   operatorController.a().whileTrue(
 *       new ManualLeftFlywheelCommand(shooter, () -> operatorController.getLeftY()));
 * </pre>
 *
 * <h2>SmartDashboard</h2>
 * <pre>
 * Shooter/LeftRotations — posicion acumulada del flywheel izquierdo [rot]
 * Shooter/LeftRPM       — velocidad del flywheel izquierdo [RPM]
 * </pre>
 */
public class ManualLeftFlywheelCommand extends Command {

    private final ShooterSubsystem shooter;
    private final DoubleSupplier   outputSupplier;

    /**
     * @param shooter        Subsistema del shooter.
     * @param outputSupplier Proveedor de salida en [-1.0, 1.0].
     *                       Se clampea internamente a FLYWHEEL_MANUAL_MAX_OUTPUT.
     */
    public ManualLeftFlywheelCommand(ShooterSubsystem shooter, DoubleSupplier outputSupplier) {
        this.shooter        = shooter;
        this.outputSupplier = outputSupplier;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // Nada especial al iniciar — el motor arranca desde parado
    }

    @Override
    public void execute() {
        // Ambos flywheels son un solo mecanismo — se controlan juntos
        shooter.runFlywheelManual(outputSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheelManual();
    }

    @Override
    public boolean isFinished() {
        // Corre hasta que se interrumpa (se suelte el boton)
        return false;
    }
}

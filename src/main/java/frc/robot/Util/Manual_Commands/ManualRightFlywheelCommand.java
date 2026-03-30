package frc.robot.Util.Manual_Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.ShooterSubsystem;

/**
 * Comando para controlar el flywheel derecho de manera manual.
 *
 * <p>Diseñado para caracterizacion y diagnostico. Permite verificar
 * independientemente que el flywheel derecho gira en la direccion correcta
 * y a la velocidad esperada antes de configurar el tiro automatico.</p>
 *
 * <p>La salida se clampea a {@code ShooterConstants.FLYWHEEL_MANUAL_MAX_OUTPUT}.</p>
 *
 * <h2>SmartDashboard</h2>
 * <pre>
 * Shooter/RightRotations — posicion acumulada del flywheel derecho [rot]
 * Shooter/RightRPM       — velocidad del flywheel derecho [RPM]
 * </pre>
 */
public class ManualRightFlywheelCommand extends Command {

    private final ShooterSubsystem shooter;
    private final DoubleSupplier   outputSupplier;

    /**
     * @param shooter        Subsistema del shooter.
     * @param outputSupplier Proveedor de salida en [-1.0, 1.0].
     *                       Se clampea internamente a FLYWHEEL_MANUAL_MAX_OUTPUT.
     */
    public ManualRightFlywheelCommand(ShooterSubsystem shooter, DoubleSupplier outputSupplier) {
        this.shooter        = shooter;
        this.outputSupplier = outputSupplier;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // Nada especial al iniciar
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
        return false;
    }
}

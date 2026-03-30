package frc.robot.Util.Manual_Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.ShooterSubsystem;

/**
 * Comando para controlar el indexer de manera manual.
 *
 * <p>Util para verificar la direccion correcta del indexer, probar la
 * alimentacion de pelotas al flywheel, y diagnosticar atascos.</p>
 *
 * <p>La salida se clampea a {@code ShooterConstants.INDEXER_MANUAL_MAX_OUTPUT}.</p>
 *
 * <h2>SmartDashboard</h2>
 * <pre>
 * Shooter/IndexerRotations  — posicion acumulada del indexer [rot del mecanismo]
 * Shooter/IndexerStatorAmps — corriente de stator [A] (detecta atascos)
 * </pre>
 */
public class ManualIndexerCommand extends Command {

    private final ShooterSubsystem shooter;
    private final DoubleSupplier   outputSupplier;

    /**
     * @param shooter        Subsistema del shooter.
     * @param outputSupplier Proveedor de salida en [-1.0, 1.0].
     *                       Se clampea internamente a INDEXER_MANUAL_MAX_OUTPUT.
     */
    public ManualIndexerCommand(ShooterSubsystem shooter, DoubleSupplier outputSupplier) {
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
        shooter.runIndexerManual(outputSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopIndexerManual();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

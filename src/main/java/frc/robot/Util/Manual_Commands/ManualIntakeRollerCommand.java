package frc.robot.Util.Manual_Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.IntakeSubsystem;

/**
 * Comando para controlar el roller del intake de manera manual.
 *
 * <p>Util para verificar direccion del roller, probar velocidades de absorcion,
 * y diagnosticar el mecanismo antes de configurar el automatismo.</p>
 *
 * <p>La salida se clampea a {@code IntakeConstants.ROLLER_MANUAL_MAX_OUTPUT}.</p>
 *
 * <h2>SmartDashboard</h2>
 * <pre>
 * Intake/RollerRotations — posicion acumulada del roller [rot del mecanismo]
 * Intake/StatorAmps      — corriente de stator [A]
 * Intake/IsRunning       — true si esta girando
 * </pre>
 */
public class ManualIntakeRollerCommand extends Command {

    private final IntakeSubsystem intake;
    private final DoubleSupplier  outputSupplier;

    /**
     * @param intake         Subsistema del intake (roller).
     * @param outputSupplier Proveedor de salida en [-1.0, 1.0].
     *                       Se clampea internamente a ROLLER_MANUAL_MAX_OUTPUT.
     */
    public ManualIntakeRollerCommand(IntakeSubsystem intake, DoubleSupplier outputSupplier) {
        this.intake          = intake;
        this.outputSupplier  = outputSupplier;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Nada especial al iniciar
    }

    @Override
    public void execute() {
        intake.runIntake(outputSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        // Siempre detener el roller al terminar
        intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

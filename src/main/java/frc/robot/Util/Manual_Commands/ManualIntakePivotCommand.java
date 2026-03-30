package frc.robot.Util.Manual_Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.IntakePivotSubsystem;

/**
 * Comando para controlar el pivot del intake de manera manual.
 *
 * <p>Activa el override de control manual en {@link IntakePivotSubsystem},
 * desactivando el PID de posicion onboard del SparkMax y aplicando una salida
 * de ciclo de trabajo directa. Al terminar, el PID se restaura automaticamente.</p>
 *
 * <p>Util para mapear el rango mecanico real del pivot (min/max rotaciones)
 * antes de configurar {@code PIVOT_DEPLOYED_POSITION_ROTATIONS} en Constants.</p>
 *
 * <p>La salida se clampea a {@code IntakeConstants.PIVOT_MANUAL_MAX_OUTPUT}.</p>
 *
 * <h2>SmartDashboard</h2>
 * <pre>
 * IntakePivot/Position       — posicion actual del pivot [rot del mecanismo]
 * IntakePivot/ManualOverride — true mientras este comando este activo
 * </pre>
 */
public class ManualIntakePivotCommand extends Command {

    private final IntakePivotSubsystem intakePivot;
    private final DoubleSupplier       outputSupplier;

    /**
     * @param intakePivot    Subsistema del pivot del intake.
     * @param outputSupplier Proveedor de salida en [-1.0, 1.0].
     *                       Se clampea internamente a PIVOT_MANUAL_MAX_OUTPUT.
     */
    public ManualIntakePivotCommand(IntakePivotSubsystem intakePivot, DoubleSupplier outputSupplier) {
        this.intakePivot    = intakePivot;
        this.outputSupplier = outputSupplier;
        addRequirements(intakePivot);
    }

    @Override
    public void initialize() {
        // Activar override antes del primer execute() para que el primer
        // ciclo de periodic() ya use el modo manual
        intakePivot.setManualOutput(0.0);
    }

    @Override
    public void execute() {
        intakePivot.setManualOutput(outputSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        // Restaurar PID — el pivot volvera a mantener su setpoint anterior
        intakePivot.disableManualOverride();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

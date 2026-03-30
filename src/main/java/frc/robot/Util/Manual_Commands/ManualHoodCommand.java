package frc.robot.Util.Manual_Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.ShooterSubsystem;

/**
 * Comando para controlar el hood (angulo de disparo) de manera manual.
 *
 * <p><b>Advertencia:</b> Este comando desactiva MotionMagic y los limites de
 * posicion. Usarlo con precaucion — solo para mapear el rango mecanico real
 * o diagnosticar el motor del hood.</p>
 *
 * <p>La salida se clampea a {@code ShooterConstants.HOOD_MANUAL_MAX_OUTPUT}
 * (valor conservador por defecto) para evitar danos.</p>
 *
 * <p>Al terminar, el hood queda en coast. Llamar un comando de home
 * inmediatamente despues si se desea restaurar el PID.</p>
 *
 * <h2>SmartDashboard</h2>
 * <pre>
 * Shooter/HoodRotations  — posicion del hood [rotaciones del mecanismo]
 * Shooter/HoodAngleDeg   — angulo del hood en grados
 * </pre>
 */
public class ManualHoodCommand extends Command {

    private final ShooterSubsystem shooter;
    private final DoubleSupplier   outputSupplier;

    /**
     * @param shooter        Subsistema del shooter.
     * @param outputSupplier Proveedor de salida en [-1.0, 1.0].
     *                       Se clampea internamente a HOOD_MANUAL_MAX_OUTPUT.
     */
    public ManualHoodCommand(ShooterSubsystem shooter, DoubleSupplier outputSupplier) {
        this.shooter        = shooter;
        this.outputSupplier = outputSupplier;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // Nada especial al iniciar — el motor ya deberia estar detenido
    }

    @Override
    public void execute() {
        shooter.runHoodManual(outputSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        // Detener el hood y dejarlo en coast.
        // No restauramos MotionMagic automaticamente para evitar movimientos
        // inesperados si el usuario termina en una posicion no calibrada.
        // El usuario debe llamar homeHood() explicitamente si lo desea.
        shooter.stopHoodManual();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

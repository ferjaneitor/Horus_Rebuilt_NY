package frc.robot.Util.Manual_Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.ShooterSubsystem;

/**
 * Comando para controlar AMBOS flywheels simultaneamente de manera manual.
 *
 * <p>Los dos flywheels giran a la vez con la misma magnitud pero signos opuestos,
 * tal como ocurre durante un disparo real (los flywheels se enfrentan entre si y
 * necesitan girar en direcciones contrarias para lanzar la pelota).</p>
 *
 * <ul>
 *   <li>Flywheel izquierdo → {@code +output}</li>
 *   <li>Flywheel derecho   → {@code -output}</li>
 * </ul>
 *
 * <p>Si fisicamente los flywheels giran en sentido incorrecto, invertir el
 * signo del {@code outputSupplier} en RobotContainer o cambiar la inversion
 * en {@link frc.robot.Constants.ShooterConstants}.</p>
 *
 * <p>La salida se clampea a {@code ShooterConstants.FLYWHEEL_MANUAL_MAX_OUTPUT}
 * dentro de cada metodo de {@link ShooterSubsystem}.</p>
 *
 * <p>Al terminar el comando ambos motores quedan en coast.</p>
 *
 * <h2>SmartDashboard (publicado por ShooterSubsystem.periodic)</h2>
 * <pre>
 * Shooter/LeftRotations  — posicion acumulada flywheel izquierdo [rot]
 * Shooter/RightRotations — posicion acumulada flywheel derecho  [rot]
 * Shooter/LeftRPM        — velocidad flywheel izquierdo [RPM]
 * Shooter/RightRPM       — velocidad flywheel derecho  [RPM]
 * </pre>
 *
 * <h2>Uso tipico (test controller)</h2>
 * <pre>
 *   testController.rightTrigger().whileTrue(
 *       new ManualFlywheelsCommand(shooter, testController::getRightTriggerAxis));
 * </pre>
 */
public class ManualFlywheelsCommand extends Command {

    private final ShooterSubsystem shooter;
    private final DoubleSupplier   outputSupplier;

    /**
     * @param shooter        Subsistema del shooter.
     * @param outputSupplier Proveedor de salida en [0.0, 1.0] (tipicamente un trigger de Xbox).
     *                       El flywheel izquierdo recibe {@code +output}
     *                       y el derecho recibe {@code -output}.
     *                       Ambos se clampean internamente a FLYWHEEL_MANUAL_MAX_OUTPUT.
     */
    public ManualFlywheelsCommand(ShooterSubsystem shooter, DoubleSupplier outputSupplier) {
        this.shooter        = shooter;
        this.outputSupplier = outputSupplier;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // Los motores arrancan desde parado; no se necesita configuracion previa.
    }

    @Override
    public void execute() {
        shooter.runFlywheelManual(outputSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheelManual();
    }

    @Override
    public boolean isFinished() {
        // Corre continuamente mientras se mantenga presionado el boton/trigger.
        return false;
    }
}

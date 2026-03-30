package frc.robot.Intake.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Intake.IntakePivotSubsystem;
import frc.robot.Constants.IntakeConstants;

/**
 * Comando que despliega el pivot del intake a la posicion de recoleccion.
 *
 * Es un InstantCommand: termina inmediatamente despues de cambiar el setpoint.
 * El PID continuo en IntakePivotSubsystem.periodic() se encarga de mover el motor.
 *
 * Diseñado para usarse con onTrue() en RobotContainer:
 *   - Un solo press del boton despliega el intake.
 *   - No es necesario mantener presionado el boton.
 *
 * TODO: Calibrar PIVOT_DEPLOYED_POSITION_ROTATIONS en Constants cuando se
 *       sepa cuantas rotaciones necesita el mecanismo para llegar a la posicion correcta.
 */
public class DeployPivotCommand extends InstantCommand {

    public DeployPivotCommand(IntakePivotSubsystem pivot) {
        super(
            () -> pivot.setSetpoint(IntakeConstants.PIVOT_DEPLOYED_POSITION_ROTATIONS),
            pivot
        );
    }
}

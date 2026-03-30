package frc.robot.Intake.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Intake.IntakePivotSubsystem;
import frc.robot.Constants.IntakeConstants;

/**
 * Comando que retrae el pivot del intake a la posicion de reposo (0 rotaciones).
 *
 * Es un InstantCommand: termina inmediatamente despues de cambiar el setpoint.
 * El PID continuo en IntakePivotSubsystem.periodic() se encarga de mover el motor.
 *
 * Diseñado para usarse con onTrue() en RobotContainer:
 *   - Un solo press del boton retrae el intake.
 *   - No es necesario mantener presionado el boton.
 */
public class RetractPivotCommand extends InstantCommand {

    public RetractPivotCommand(IntakePivotSubsystem pivot) {
        super(
            () -> pivot.setSetpoint(IntakeConstants.PIVOT_RETRACTED_POSITION_ROTATIONS),
            pivot
        );
    }
}

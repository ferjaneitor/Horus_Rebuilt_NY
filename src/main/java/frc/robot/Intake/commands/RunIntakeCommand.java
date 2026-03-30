package frc.robot.Intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.IntakeSubsystem;

/**
 * Comando que corre el roller del intake mientras este activo.
 *
 * Diseñado para usarse con whileTrue() en RobotContainer:
 *   - El roller corre mientras el boton este presionado.
 *   - Al soltar el boton, el comando es interrumpido y end() detiene el roller.
 *
 * No hace falta presionar y sostener un toggle — esto es intencional para
 * darle control inmediato al driver.
 */
public class RunIntakeCommand extends Command {

    private final IntakeSubsystem intake;

    public RunIntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Nada especial al iniciar
    }

    @Override
    public void execute() {
        intake.runIntake();
    }

    @Override
    public void end(boolean interrupted) {
        // Se llama tanto si el comando termina normalmente como si es interrumpido.
        // Siempre detenemos el motor para no dejar el roller girando solo.
        intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        // Este comando no termina solo — corre hasta que sea interrumpido
        // (cuando el driver suelte el boton).
        return false;
    }
}

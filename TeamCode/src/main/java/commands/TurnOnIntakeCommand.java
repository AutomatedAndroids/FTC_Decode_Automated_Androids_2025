package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import subsystems.IntakeSubsystem;

/**
 * Command to turn on the intake.
 */
public class TurnOnIntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public TurnOnIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.turnOnIntake();
    }

    @Override
    public boolean isFinished() {
        return true; // Instant command
    }
}

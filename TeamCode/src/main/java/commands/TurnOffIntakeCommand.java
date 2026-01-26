package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import subsystems.IntakeSubsystem;

/**
 * Command to turn off the intake.
 */
public class TurnOffIntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public TurnOffIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.turnOffIntake();
    }

    @Override
    public boolean isFinished() {
        return true; // Instant command
    }
}

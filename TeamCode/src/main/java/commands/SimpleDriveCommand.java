package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import subsystems.MecanumDriveSubsystem;

/**
 * Simple command that drives the robot in a direction for a specified duration.
 */
public class SimpleDriveCommand extends CommandBase {
    private final MecanumDriveSubsystem driveSubsystem;
    private final double forward;
    private final double strafe;
    private final double turn;
    private final double durationSeconds;
    private double startTime;
    private boolean initialized = false;

    public SimpleDriveCommand(
            MecanumDriveSubsystem driveSubsystem,
            double forward,
            double strafe,
            double turn,
            double durationSeconds) {
        this.driveSubsystem = driveSubsystem;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        this.durationSeconds = durationSeconds;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis() / 1000.0;
        initialized = true;
    }

    @Override
    public void execute() {
        if (initialized) {
            driveSubsystem.setDrivePowers(forward, strafe, turn);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setDrivePowers(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        if (!initialized) return false;
        double elapsed = (System.currentTimeMillis() / 1000.0) - startTime;
        return elapsed >= durationSeconds;
    }
}

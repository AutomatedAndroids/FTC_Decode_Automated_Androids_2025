package commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import drive.MecanumDrive;
import subsystems.MecanumSubsystem;

/**
 * Command that drives to a target position and completes when arrived.
 * Uses the time-based motion profile system.
 */
public class DriveToPositionCommand extends CommandBase {
    private final MecanumSubsystem drive;
    private final double targetXTicks;
    private final double targetYTicks;
    private final double targetHeadingRad;
    private boolean isComplete = false;

    public DriveToPositionCommand(MecanumSubsystem drive, Pose2d targetPose) {
        this.drive = drive;
        // Convert AutoConstants (Inches) to Ticks for the drive method
        this.targetXTicks = targetPose.position.x * MecanumDrive.TICKS_PER_INCH;
        this.targetYTicks = targetPose.position.y * MecanumDrive.TICKS_PER_INCH;
        this.targetHeadingRad = targetPose.heading.toDouble();
        addRequirements(drive);
    }

    public DriveToPositionCommand(MecanumSubsystem drive, double xTicks, double yTicks, double headingRad) {
        this.drive = drive;
        this.targetXTicks = xTicks;
        this.targetYTicks = yTicks;
        this.targetHeadingRad = headingRad;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        isComplete = false;
    }

    @Override
    public void execute() {
        // driveToPosition returns true when complete
        isComplete = drive.driveToPosition(targetXTicks, targetYTicks, targetHeadingRad);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop motors when command ends
        drive.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return isComplete;
    }
}


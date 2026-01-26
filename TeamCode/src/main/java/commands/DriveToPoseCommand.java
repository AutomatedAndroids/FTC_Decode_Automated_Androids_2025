package commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import subsystems.MecanumDriveSubsystem;

/**
 * Command that drives to a target pose using simple PID-like control.
 * Calculates direction and drives until close to target.
 */
public class DriveToPoseCommand extends CommandBase {
    private final MecanumDriveSubsystem driveSubsystem;
    private final Pose2d targetPose;
    private final double tolerance; // inches
    private final double headingTolerance; // radians
    
    private static final double P_GAIN = 0.05;
    private static final double MAX_POWER = 0.6;
    
    public DriveToPoseCommand(MecanumDriveSubsystem driveSubsystem, Pose2d targetPose) {
        this(driveSubsystem, targetPose, 2.0, Math.toRadians(5));
    }
    
    public DriveToPoseCommand(MecanumDriveSubsystem driveSubsystem, Pose2d targetPose, double tolerance, double headingTolerance) {
        this.driveSubsystem = driveSubsystem;
        this.targetPose = targetPose;
        this.tolerance = tolerance;
        this.headingTolerance = headingTolerance;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();
        
        // Calculate errors
        double xError = targetPose.position.x - currentPose.position.x;
        double yError = targetPose.position.y - currentPose.position.y;
        double headingError = targetPose.heading.toDouble() - currentPose.heading.toDouble();
        
        // Normalize heading error to [-pi, pi]
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError <= -Math.PI) headingError += 2 * Math.PI;
        
        double distance = Math.hypot(xError, yError);
        
        // Convert world-frame errors to robot-frame
        double cosHeading = Math.cos(currentPose.heading.toDouble());
        double sinHeading = Math.sin(currentPose.heading.toDouble());
        
        double forward = (xError * cosHeading + yError * sinHeading) * P_GAIN;
        double strafe = (-xError * sinHeading + yError * cosHeading) * P_GAIN;
        double turn = headingError * P_GAIN * 2; // Scale up heading gain
        
        // Limit power
        double maxCommand = Math.max(Math.abs(forward), Math.max(Math.abs(strafe), Math.abs(turn)));
        if (maxCommand > MAX_POWER) {
            forward = (forward / maxCommand) * MAX_POWER;
            strafe = (strafe / maxCommand) * MAX_POWER;
            turn = (turn / maxCommand) * MAX_POWER;
        }
        
        // Apply deadband for small errors
        if (distance < 0.5) {
            forward *= distance / 0.5;
            strafe *= distance / 0.5;
        }
        if (Math.abs(headingError) < Math.toRadians(2)) {
            turn *= Math.abs(headingError) / Math.toRadians(2);
        }
        
        driveSubsystem.setDrivePowers(forward, strafe, turn);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setDrivePowers(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = driveSubsystem.getPose();
        double xError = targetPose.position.x - currentPose.position.x;
        double yError = targetPose.position.y - currentPose.position.y;
        double headingError = targetPose.heading.toDouble() - currentPose.heading.toDouble();
        
        // Normalize heading error
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError <= -Math.PI) headingError += 2 * Math.PI;
        
        double distance = Math.hypot(xError, yError);
        return distance < tolerance && Math.abs(headingError) < headingTolerance;
    }
}

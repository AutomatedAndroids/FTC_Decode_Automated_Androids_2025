package opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import commands.DriveToPoseCommand;
import subsystems.MecanumDriveSubsystem;
import subsystems.VisionSubsystem;

/**
 * Test autonomous to verify encoder and IMU configuration.
 * 
 * Robot should start at APEX position.
 * Test sequence:
 * 1. Move 2 inches forward
 * 2. Move 2 inches right
 * 3. Move 2 inches left
 * 
 * Watch the telemetry to see if the robot's reported position matches
 * the actual movement. If movements are in wrong directions, check:
 * - Encoder directions (may need to reverse some encoders)
 * - IMU orientation (logoFacingDirection and usbFacingDirection)
 */
@Autonomous(name = "Test Localization", group = "Test")
public class TestLocalizationAuto extends BaseAuto {
    
    @Override
    protected Pose2d getStartPose() {
        // Start at BLUE_APEX position
        return AutoConstants.BLUE_APEX;
    }

    @Override
    protected void buildAuto() {
        // This test uses direct pose updates, so we'll handle it in runOpMode
        // For now, just schedule an empty command - the test logic is in runOpMode
    }
    
    @Override
    public void runOpMode() {
        initialize();
        
        telemetry.addData("Status", "Initialized at APEX");
        telemetry.addData("Start X", String.format("%.2f", getStartPose().position.x));
        telemetry.addData("Start Y", String.format("%.2f", getStartPose().position.y));
        telemetry.addData("Start Heading", String.format("%.2f", Math.toDegrees(getStartPose().heading.toDouble())));
        telemetry.update();
        
        waitForStart();
        
        if (isStopRequested()) return;
        
        // Get current pose
        Pose2d currentPose = drive.getPose();
        telemetry.addData("Current X", String.format("%.2f", currentPose.position.x));
        telemetry.addData("Current Y", String.format("%.2f", currentPose.position.y));
        telemetry.addData("Current Heading", String.format("%.2f", Math.toDegrees(currentPose.heading.toDouble())));
        telemetry.update();
        sleep(1000);
        
        // Test 1: Move 2 inches forward (in robot's forward direction)
        telemetry.addData("Test", "Moving 2 inches FORWARD");
        telemetry.update();
        
        // Calculate target: 2 inches forward from current position
        // Forward is along the robot's current heading
        double forwardX = currentPose.position.x + 2.0 * Math.cos(currentPose.heading.toDouble());
        double forwardY = currentPose.position.y + 2.0 * Math.sin(currentPose.heading.toDouble());
        Pose2d forwardTarget = new Pose2d(forwardX, forwardY, currentPose.heading.toDouble());
        
        // Drive to forward position
        schedule(new DriveToPoseCommand(drive, forwardTarget));
        while (opModeIsActive() && !isStopRequested()) {
            run(); // Run command scheduler
            Pose2d pose = drive.getPose();
            telemetry.addData("Moving to", String.format("X: %.2f, Y: %.2f", forwardX, forwardY));
            telemetry.addData("Current", String.format("X: %.2f, Y: %.2f, H: %.2f", 
                pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble())));
            telemetry.update();
            sleep(50);
            if (!opModeIsActive()) break;
        }
        
        // Stop and wait
        drive.setDrivePowers(0, 0, 0);
        sleep(1000);
        
        currentPose = drive.getPose();
        telemetry.addData("After Forward", String.format("X: %.2f, Y: %.2f", currentPose.position.x, currentPose.position.y));
        telemetry.update();
        sleep(2000);
        
        // Test 2: Move 2 inches RIGHT (perpendicular to robot's forward direction)
        telemetry.addData("Test", "Moving 2 inches RIGHT");
        telemetry.update();
        
        // Right is 90 degrees clockwise from forward
        double rightX = currentPose.position.x + 2.0 * Math.cos(currentPose.heading.toDouble() - Math.PI / 2);
        double rightY = currentPose.position.y + 2.0 * Math.sin(currentPose.heading.toDouble() - Math.PI / 2);
        Pose2d rightTarget = new Pose2d(rightX, rightY, currentPose.heading.toDouble());
        
        // Drive to right position
        schedule(new DriveToPoseCommand(drive, rightTarget));
        while (opModeIsActive() && !isStopRequested()) {
            run(); // Run command scheduler
            Pose2d pose = drive.getPose();
            telemetry.addData("Moving to", String.format("X: %.2f, Y: %.2f", rightX, rightY));
            telemetry.addData("Current", String.format("X: %.2f, Y: %.2f, H: %.2f", 
                pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble())));
            telemetry.update();
            sleep(50);
            if (!opModeIsActive()) break;
        }
        
        // Stop and wait
        drive.setDrivePowers(0, 0, 0);
        sleep(1000);
        
        currentPose = drive.getPose();
        telemetry.addData("After Right", String.format("X: %.2f, Y: %.2f", currentPose.position.x, currentPose.position.y));
        telemetry.update();
        sleep(2000);
        
        // Test 3: Move 2 inches LEFT (opposite of right)
        telemetry.addData("Test", "Moving 2 inches LEFT");
        telemetry.update();
        
        // Left is 90 degrees counterclockwise from forward
        double leftX = currentPose.position.x + 2.0 * Math.cos(currentPose.heading.toDouble() + Math.PI / 2);
        double leftY = currentPose.position.y + 2.0 * Math.sin(currentPose.heading.toDouble() + Math.PI / 2);
        Pose2d leftTarget = new Pose2d(leftX, leftY, currentPose.heading.toDouble());
        
        // Drive to left position
        schedule(new DriveToPoseCommand(drive, leftTarget));
        while (opModeIsActive() && !isStopRequested()) {
            run(); // Run command scheduler
            Pose2d pose = drive.getPose();
            telemetry.addData("Moving to", String.format("X: %.2f, Y: %.2f", leftX, leftY));
            telemetry.addData("Current", String.format("X: %.2f, Y: %.2f, H: %.2f", 
                pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble())));
            telemetry.update();
            sleep(50);
            if (!opModeIsActive()) break;
        }
        
        // Stop
        drive.setDrivePowers(0, 0, 0);
        
        currentPose = drive.getPose();
        telemetry.addData("Final Position", String.format("X: %.2f, Y: %.2f, H: %.2f", 
            currentPose.position.x, currentPose.position.y, Math.toDegrees(currentPose.heading.toDouble())));
        telemetry.addData("Test", "Complete!");
        telemetry.update();
        
        // Keep running to see final position
        while (opModeIsActive() && !isStopRequested()) {
            run();
            telemetry.update();
            sleep(100);
        }
        
        // Cleanup
        if (vision != null) {
            vision.stop();
        }
        reset();
    }
}

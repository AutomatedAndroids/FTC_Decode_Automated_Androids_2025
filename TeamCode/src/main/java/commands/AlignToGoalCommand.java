package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry; // Import added
import subsystems.MecanumSubsystem;
import util.ShootingPresets;

public class AlignToGoalCommand extends CommandBase {
    private subsystems.MecanumSubsystem drive;
    private final Limelight3A limelight;
    private final GamepadEx gamepad;
    private final Telemetry telemetry; // Telemetry added

    // Configurable Target Parameters
    private final double targetDistance;
    private final double targetHeight;
    private final int targetPipeline;

    // Tolerances
    private static final double DISTANCE_TOLERANCE = 1.0; // inches
    private static final double HEADING_TOLERANCE = 1.0; // degrees

    // State variables
    private double currentDistance;
    private double headingError;
    private boolean hasTarget;

    public AlignToGoalCommand(MecanumSubsystem drive, Limelight3A limelight, GamepadEx gamepad, Telemetry telemetry,
                              double targetDistance, double targetHeight, int targetPipeline) {
        this.drive = drive;
        this.limelight = limelight;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.targetDistance = targetDistance;
        this.targetHeight = targetHeight;
        this.targetPipeline = targetPipeline;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Switch Limelight to the correct pipeline/mode for this specific goal
        limelight.pipelineSwitch(targetPipeline);
        
        // Reset state
        currentDistance = targetDistance; // Default to target so we don't jump if no target seen immediately
        headingError = 0;
        hasTarget = false;
    }

    @Override
    public void execute() {
        // Default to "we are there" if no target found to prevent sporadic movement
        // But we keep the LAST known good values if we want persistence, 
        // OR we default to safe values.
        // Here we default to "safe" (no error) every loop if no target.
        currentDistance = targetDistance; 
        headingError = 0;
        hasTarget = false;

        if (limelight.getLatestResult() != null && limelight.getLatestResult().isValid()) {
            hasTarget = true;
            headingError = limelight.getLatestResult().getTx();
            double ty = limelight.getLatestResult().getTy();

            // Calculate Distance: d = (TargetH - CameraH) / tan(CameraAngle + ty)
            double heightDiff = targetHeight - ShootingPresets.CAMERA_HEIGHT;
            double totalAngle = Math.toRadians(ShootingPresets.CAMERA_ANGLE + ty);

            if (Math.tan(totalAngle) != 0) {
                double cameraDist = heightDiff / Math.tan(totalAngle);
                currentDistance = cameraDist + ShootingPresets.CAMERA_OFFSET;
            }
        }
        
        telemetry.addData("Align/TargetPipeline", targetPipeline);
        telemetry.addData("Align/HasTarget", hasTarget);
        telemetry.addData("Align/HeadingError", headingError);
        telemetry.addData("Align/CurrentDist", currentDistance);
        telemetry.addData("Align/TargetDist", targetDistance);
        telemetry.addData("Align/ErrorDist", targetDistance - currentDistance);
        // Telemetry update is handled by OpMode or we can force it here if needed, 
        // but typically we just add data.

        // Pass calculated values to the subsystem with manual overrides
        // Inputs: strafe, forward, turn, currentDist, targetDist, headingError
        // Note: -getLeftY() for forward because gamepad Y is inverted
        drive.driveToGoal(gamepad.getLeftX(), -gamepad.getLeftY(), gamepad.getRightX(), currentDistance, targetDistance, headingError);
    }

    @Override
    public boolean isFinished() {
        if (hasTarget) {
            double distError = Math.abs(targetDistance - currentDistance);
            double headError = Math.abs(headingError);
            return headError < HEADING_TOLERANCE && distError < DISTANCE_TOLERANCE;
        }
        return false;
    }
}
package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import subsystems.MecanumSubsystem;
import util.ShootingPresets;

public class AlignToGoalCommand extends CommandBase {
    private subsystems.MecanumSubsystem drive;
    private final Limelight3A limelight;
    private final GamepadEx gamepad;

    // Configurable Target Parameters
    private final double targetDistance;
    private final double targetHeight;
    private final int targetPipeline;

    public AlignToGoalCommand(MecanumSubsystem drive, Limelight3A limelight, GamepadEx gamepad,
                              double targetDistance, double targetHeight, int targetPipeline) {
        this.drive = drive;
        this.limelight = limelight;
        this.gamepad = gamepad;
        this.targetDistance = targetDistance;
        this.targetHeight = targetHeight;
        this.targetPipeline = targetPipeline;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Switch Limelight to the correct pipeline/mode for this specific goal
        limelight.pipelineSwitch(targetPipeline);
    }

    @Override
    public void execute() {
        double headingError = 0;
        double currentDistance = targetDistance; // Default to "we are there"

        if (limelight.getLatestResult() != null && limelight.getLatestResult().isValid()) {
            headingError = limelight.getLatestResult().getTx();
            double ty = limelight.getLatestResult().getTy();

            // Calculate Distance: d = (TargetH - CameraH) / tan(CameraAngle + ty)
            double heightDiff = targetHeight - ShootingPresets.CAMERA_HEIGHT;
            double totalAngle = Math.toRadians(ShootingPresets.CAMERA_ANGLE + ty);

            if (Math.tan(totalAngle) != 0) {
                currentDistance = heightDiff / Math.tan(totalAngle);
            }
        }

        // Pass calculated values to the subsystem
        drive.driveToGoal(gamepad.getLeftX(), currentDistance, targetDistance, headingError);
    }
}
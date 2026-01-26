package opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import commands.DriveToPoseCommand;

/**
 * Example autonomous opmode showing how to use BaseAuto.
 * Starts at BLUE_BACK_START and drives to BLUE_APEX.
 */
@Autonomous(name = "Blue Auto Back Simple Park", group = "Autonomous")
public class BlueBackSimple extends BaseAuto {

    @Override
    protected Pose2d getStartPose() {
        // Start at the blue back starting position
        return AutoConstants.BLUE_BACK_START;
    }

    @Override
    protected void buildAuto() {
        // Build your autonomous sequence using commands
        schedule(new SequentialCommandGroup(
                // Drive from BLUE_BACK_START to BLUE_APEX
                new DriveToPoseCommand(drive, AutoConstants.BLUE_APEX)
        ));
    }
}

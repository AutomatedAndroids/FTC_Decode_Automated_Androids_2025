package opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import commands.DriveToPositionCommand;

@Autonomous(name = "Blue Auto Back Simple Park")
public class BlueBackSimple extends BaseAuto {

    @Override
    protected Pose2d getStartPose() {
        return AutoConstants.BLUE_BACK_START;
    }

    @Override
    protected void buildAuto() {
        // Simple sequence: Drive to target position
        // Uses time-based motion profile drive commands
        
        schedule(new SequentialCommandGroup(
                // 1. Drive to target position (50 inches forward)
                new DriveToPositionCommand(drive, new Pose2d(0, 50, 0))
        ));
    }
}

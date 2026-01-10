package opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import commands.SimpleMoveCommand;

@Autonomous(name = "Blue Auto Back Simple Park")
public class BlueBackSimple extends BaseAuto {

    @Override
    protected Pose2d getStartPose() {
        return AutoConstants.BLUE_BACK_START;
    }

    @Override
    protected void buildAuto() {
        // Simple sequence: Drive to score position -> Shoot
        // Uses "Stupid Drive" blocking commands
        
        schedule(new SequentialCommandGroup(
                // 1. Drive to scoring position (Start from Pre-defined Start Pose)
                new InstantCommand(() -> drive.driveToPositionStupid(this, new Pose2d(0, 50, 0)))
        ));
    }
}

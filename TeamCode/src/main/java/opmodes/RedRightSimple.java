package opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Right - Simple (Park)")
public class RedRightSimple extends BaseAuto {

    @Override
    protected Pose2d getStartPose() {
        return AutoConstants.RED_RIGHT_START;
    }

    @Override
    protected void buildAuto() {
        Action trajectory = drive.actionBuilder(getStartPose())
                .strafeTo(AutoConstants.RED_PARK.position)
                .build();

        schedule(new SequentialCommandGroup(
                runAction(trajectory)
        ));
    }
}


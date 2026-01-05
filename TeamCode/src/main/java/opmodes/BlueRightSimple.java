package opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Right - Simple (Park)")
public class BlueRightSimple extends BaseAuto {

    @Override
    protected Pose2d getStartPose() {
        return AutoConstants.BLUE_RIGHT_START;
    }

    @Override
    protected void buildAuto() {
        Action trajectory = drive.actionBuilder(getStartPose())
                .strafeTo(AutoConstants.BLUE_PARK.position)
                .build();

        schedule(new SequentialCommandGroup(
                runAction(trajectory)
        ));
    }
}


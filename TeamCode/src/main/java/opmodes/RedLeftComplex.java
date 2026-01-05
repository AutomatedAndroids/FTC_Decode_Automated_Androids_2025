package opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Left - Complex (Shoot)")
public class RedLeftComplex extends BaseAuto {

    @Override
    protected Pose2d getStartPose() {
        return AutoConstants.RED_LEFT_START;
    }

    @Override
    protected void buildAuto() {
        Action driveToShoot = drive.actionBuilder(getStartPose())
                .strafeTo(AutoConstants.RED_SHOOT_POS.position)
                .turnTo(AutoConstants.RED_SHOOT_POS.heading)
                .build();
                
        Action driveToPark = drive.actionBuilder(AutoConstants.RED_SHOOT_POS)
                .strafeTo(AutoConstants.RED_PARK.position)
                .build();

        schedule(new SequentialCommandGroup(
                // Drive to shooting position
                runAction(driveToShoot),
                
                // Shoot 3 balls
                new InstantCommand(() -> {
                    if (shooter != null) shooter.shoot_far();
                }),
                new WaitCommand(1000),
                new InstantCommand(() -> {
                    if (shooter != null) shooter.feed();
                }),
                new WaitCommand(2000),
                new InstantCommand(() -> {
                    if (shooter != null) {
                        shooter.stopFeeding();
                        shooter.stopFlywheels();
                    }
                }),
                
                // Park
                runAction(driveToPark)
        ));
    }
}


package opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import subsystems.MecanumSubsystem;

@Autonomous(name = "Blue Auto - Pinpoint")
public class AutoOpModePathPlan extends CommandOpMode {

    private MecanumSubsystem driveSubsystem;
    // private IntakeSubsystem intakeSubsystem;
    // private ShooterSubsystem shooterSubsystem;

    @Override
    public void initialize() {
        // 1. Define Starting Pose (e.g., against the wall)
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        // 2. Init Subsystem
        driveSubsystem = new MecanumSubsystem(hardwareMap, startPose, true);

        // (Initialize other subsystems here...)

        // 3. Create the Trajectory Action
        // Example: Move forward 24 inches, Turn 90 degrees
        com.acmerobotics.roadrunner.Action trajectory = driveSubsystem.actionBuilder(startPose)
                .lineToX(24)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .lineToX(0)
                .build();

        // 4. Schedule the Command
        // We wrap the RR Action in an InstantCommand + Actions.runBlocking
        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> Actions.runBlocking(trajectory)),
                new WaitCommand(1000) // Example wait
                // Add shooter commands here
        ));
    }
}
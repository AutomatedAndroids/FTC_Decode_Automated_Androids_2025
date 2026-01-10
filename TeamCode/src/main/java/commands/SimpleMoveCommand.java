package commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import subsystems.MecanumSubsystem;

public class SimpleMoveCommand extends CommandBase {
    private final MecanumSubsystem drive;
    private final Pose2d target;
    private final LinearOpMode opMode;

    public SimpleMoveCommand(MecanumSubsystem drive, Pose2d target, LinearOpMode opMode) {
        this.drive = drive;
        this.target = target;
        this.opMode = opMode;
        // Do NOT require drive subsystem if we want other things to run?
        // But driveToPositionStupid is blocking, so other things won't run anyway.
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Blocking call happens here or execute?
        // In Command-based, initialize should be quick.
        // But since we want to block until done (for sequential groups), 
        // and we are using a custom blocking method, we can do it in initialize 
        // if we accept that the loop will pause here.
        // However, CommandOpMode calls run() on scheduler in loop.
        // If we block initialize, the scheduler is blocked.
        // This is effectively what the user asked for: "Run until encoder reads right thing".
        
        drive.driveToPositionStupid(opMode, target);
    }

    @Override
    public boolean isFinished() {
        return true; // Since the move is blocking in initialize, we are done immediately after it returns.
    }
}


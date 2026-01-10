package commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import drive.MecanumDrive;
import subsystems.MecanumSubsystem;

public class OdoAlignCommand extends CommandBase {
    private final MecanumSubsystem drive;
    private final java.util.function.Supplier<Pose2d> targetSupplier;

    public OdoAlignCommand(MecanumSubsystem drive, Pose2d targetPose) {
        this.drive = drive;
        this.targetSupplier = () -> targetPose;
        addRequirements(drive);
    }

    public OdoAlignCommand(MecanumSubsystem drive, java.util.function.Supplier<Pose2d> targetSupplier) {
        this.drive = drive;
        this.targetSupplier = targetSupplier;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        Pose2d targetPose = targetSupplier.get();
        // Convert AutoConstants (Inches) to Ticks for the drive method
        double targetXTicks = targetPose.position.x * MecanumDrive.TICKS_PER_INCH;
        double targetYTicks = targetPose.position.y * MecanumDrive.TICKS_PER_INCH;
        double targetHeading = targetPose.heading.toDouble();

        drive.driveToPosition(targetXTicks, targetYTicks, targetHeading);
    }
}


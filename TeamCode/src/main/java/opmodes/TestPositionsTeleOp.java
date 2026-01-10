package opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import commands.OdoAlignCommand;
import commands.TeleOpDriveCommand;
import subsystems.MecanumSubsystem;

@TeleOp(name = "Test Auto Positions", group = "Test")
public class TestPositionsTeleOp extends CommandOpMode {

    private MecanumSubsystem drive;
    private GamepadEx driverOp;
    
    // List of positions to cycle through
    private List<Pose2d> positions;
    private List<String> positionNames;
    private int currentIndex = 0;

    @Override
    public void initialize() {
        drive = new MecanumSubsystem(hardwareMap, new Pose2d(0, 0, 0), true, telemetry);
        driverOp = new GamepadEx(gamepad1);

        drive.setDefaultCommand(new TeleOpDriveCommand(drive, driverOp));

        // Load positions from AutoConstants
        positions = new ArrayList<>(Arrays.asList(
                AutoConstants.BLUE_SCORE_CLOSE,
                AutoConstants.BLUE_SCORE_FAR,
                AutoConstants.RED_SCORE_CLOSE,
                AutoConstants.RED_SCORE_FAR,
                AutoConstants.BLUE_PARK,
                AutoConstants.RED_PARK
        ));
        
        positionNames = new ArrayList<>(Arrays.asList(
                "BLUE_CLOSE",
                "BLUE_FAR",
                "RED_CLOSE",
                "RED_FAR",
                "BLUE_PARK",
                "RED_PARK"
        ));

        // CONTROLS:
        // A (Hold): Drive to current selected position
        // D-Pad Up: Next Position
        // D-Pad Down: Previous Position
        // Start: Reset Pose to 0,0,0

        driverOp.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(new OdoAlignCommand(drive, () -> positions.get(currentIndex)));
        
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    currentIndex = (currentIndex + 1) % positions.size();
                }));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    currentIndex--;
                    if (currentIndex < 0) currentIndex = positions.size() - 1;
                }));

        driverOp.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(() -> drive.setPose(0, 0, 0)));
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Selected Position", positionNames.get(currentIndex));
        Pose2d target = positions.get(currentIndex);
        telemetry.addData("Target X", target.position.x);
        telemetry.addData("Target Y", target.position.y);
        telemetry.addData("Target H (deg)", Math.toDegrees(target.heading.toDouble()));
        
        Pose2d current = drive.getPose();
        telemetry.addData("Current X", current.position.x);
        telemetry.addData("Current Y", current.position.y);
        telemetry.addData("Current H", Math.toDegrees(current.heading.toDouble()));
        
        double xError = target.position.x - current.position.x;
        double yError = target.position.y - current.position.y;
        telemetry.addData("Error X", xError);
        telemetry.addData("Error Y", yError);
        
        telemetry.update();
    }
}


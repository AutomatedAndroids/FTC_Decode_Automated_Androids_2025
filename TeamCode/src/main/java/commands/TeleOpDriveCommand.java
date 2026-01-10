package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import subsystems.MecanumSubsystem;

public class TeleOpDriveCommand extends CommandBase {
    private final MecanumSubsystem drive;
    private final GamepadEx gamepad;

    public TeleOpDriveCommand(MecanumSubsystem drive, GamepadEx gamepad) {
        this.drive = drive;
        this.gamepad = gamepad;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        // Standard Robot Centric Drive
        // Use gamepad.getLeftX() for Field Centric math if desired
        // INVERTED Y because Gamepad Y is negative up
        double strafe = gamepad.getLeftX();
        double forward = -gamepad.getLeftY();
        double turn = -gamepad.getRightX();
        
        drive.drive(strafe, forward, turn);
    }              }
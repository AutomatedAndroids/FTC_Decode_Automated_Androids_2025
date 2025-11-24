package opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import commands.AlignToGoalCommand;
import commands.TeleOpDriveCommand;
import subsystems.MecanumSubsystem;
import util.ShootingPresets;

@TeleOp
public class MainTeleOp extends CommandOpMode {

    private Limelight3A limelight;
    private MecanumSubsystem drive;

    @Override
    public void initialize() {
        // 1. Hardware Init
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        drive = new MecanumSubsystem(hardwareMap, new Pose2d(0,0,0));
        GamepadEx driverOp = new GamepadEx(gamepad1);

        // 2. Default Drive
        drive.setDefaultCommand(new TeleOpDriveCommand(drive, driverOp));

        // --- DEFINE THE 6 MODES ---

        // GOAL 1 COMMANDS (Default Goal)
        AlignToGoalCommand goal1_Close = new AlignToGoalCommand(drive, limelight, driverOp,
                ShootingPresets.DIST_CLOSE, ShootingPresets.GOAL_1_HEIGHT, ShootingPresets.GOAL_1_PIPELINE);

        AlignToGoalCommand goal1_Mid = new AlignToGoalCommand(drive, limelight, driverOp,
                ShootingPresets.DIST_MID, ShootingPresets.GOAL_1_HEIGHT, ShootingPresets.GOAL_1_PIPELINE);

        AlignToGoalCommand goal1_Far = new AlignToGoalCommand(drive, limelight, driverOp,
                ShootingPresets.DIST_FAR, ShootingPresets.GOAL_1_HEIGHT, ShootingPresets.GOAL_1_PIPELINE);

        // GOAL 2 COMMANDS (Alternative Goal)
        AlignToGoalCommand goal2_Close = new AlignToGoalCommand(drive, limelight, driverOp,
                ShootingPresets.DIST_CLOSE, ShootingPresets.GOAL_2_HEIGHT, ShootingPresets.GOAL_2_PIPELINE);

        AlignToGoalCommand goal2_Mid = new AlignToGoalCommand(drive, limelight, driverOp,
                ShootingPresets.DIST_MID, ShootingPresets.GOAL_2_HEIGHT, ShootingPresets.GOAL_2_PIPELINE);

        AlignToGoalCommand goal2_Far = new AlignToGoalCommand(drive, limelight, driverOp,
                ShootingPresets.DIST_FAR, ShootingPresets.GOAL_2_HEIGHT, ShootingPresets.GOAL_2_PIPELINE);


        // 3. Button Bindings
        // Logic:
        // - Buttons A, B, X select Distance (Close, Mid, Far).
        // - Holding Left Bumper (LB) shifts to "Goal 2".

        // --- CLOSE DISTANCE (Button A) ---
        driverOp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> {
                    if (driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                        schedule(goal2_Close); // LB + A = Goal 2 Close
                    } else {
                        schedule(goal1_Close); // Just A = Goal 1 Close
                    }
                });

        // --- MID DISTANCE (Button B) ---
        driverOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> {
                    if (driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                        schedule(goal2_Mid); // LB + B = Goal 2 Mid
                    } else {
                        schedule(goal1_Mid); // Just B = Goal 1 Mid
                    }
                });

        // --- FAR DISTANCE (Button X) ---
        driverOp.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> {
                    if (driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                        schedule(goal2_Far); // LB + X = Goal 2 Far
                    } else {
                        schedule(goal1_Far); // Just X = Goal 1 Far
                    }
                });

        // OPTIONAL: Cancel alignment if the user moves the right stick (Manual Override)
        // You might need a custom trigger or just press another button to cancel.
    }
}
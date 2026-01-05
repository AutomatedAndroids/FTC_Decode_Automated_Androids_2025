package opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import commands.AlignToGoalCommand;
import commands.TeleOpDriveCommand;
import subsystems.IntakeSubsystem;
import subsystems.MecanumSubsystem;
import subsystems.ShooterSubsystem;
import util.ShootingPresets;

@TeleOp
public class MainTeleOp extends CommandOpMode {

    private Limelight3A limelight;
    private MecanumSubsystem drive;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }

    @Override
    public void initialize() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        // 1. Initialize Subsystems
        drive = new MecanumSubsystem(hardwareMap, new Pose2d(0,0,0), true);
        
        try {
            CRServo leftFeeder = new CRServo(hardwareMap, "leftFeeder");
            CRServo rightFeeder = new CRServo(hardwareMap, "rightFeeder");
            MotorEx shooterMotor = new MotorEx(hardwareMap, "shooter");
            shooter = new ShooterSubsystem(leftFeeder, rightFeeder, shooterMotor, telemetry);
        } catch (Exception e) {
            telemetry.addData("Warning", "Shooter failed to init");
        }

        try {
            Motor intakeMotor = new Motor(hardwareMap, "intake");
            Servo sortArm = hardwareMap.get(Servo.class, "sortArm");
            intake = new IntakeSubsystem(intakeMotor, sortArm, telemetry);
        } catch (Exception e) {
            telemetry.addData("Warning", "Intake failed to init");
        }

        // 2. Initialize Gamepads
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx operatorOp = new GamepadEx(gamepad2);

        // 3. Default Drive
        drive.setDefaultCommand(new TeleOpDriveCommand(drive, driverOp));

        // --- DEFINE THE 6 MODES ---

        // GOAL 1 COMMANDS (Default Goal)
        AlignToGoalCommand goal1_Close = new AlignToGoalCommand(drive, limelight, driverOp, telemetry,
                ShootingPresets.DIST_CLOSE, ShootingPresets.GOAL_1_HEIGHT, ShootingPresets.GOAL_1_PIPELINE);

        AlignToGoalCommand goal1_Mid = new AlignToGoalCommand(drive, limelight, driverOp, telemetry,
                ShootingPresets.DIST_MID, ShootingPresets.GOAL_1_HEIGHT, ShootingPresets.GOAL_1_PIPELINE);

        AlignToGoalCommand goal1_Far = new AlignToGoalCommand(drive, limelight, driverOp, telemetry,
                ShootingPresets.DIST_FAR, ShootingPresets.GOAL_1_HEIGHT, ShootingPresets.GOAL_1_PIPELINE);

        // GOAL 2 COMMANDS (Alternative Goal)
        AlignToGoalCommand goal2_Close = new AlignToGoalCommand(drive, limelight, driverOp, telemetry,
                ShootingPresets.DIST_CLOSE, ShootingPresets.GOAL_2_HEIGHT, ShootingPresets.GOAL_2_PIPELINE);

        AlignToGoalCommand goal2_Mid = new AlignToGoalCommand(drive, limelight, driverOp, telemetry,
                ShootingPresets.DIST_MID, ShootingPresets.GOAL_2_HEIGHT, ShootingPresets.GOAL_2_PIPELINE);

        AlignToGoalCommand goal2_Far = new AlignToGoalCommand(drive, limelight, driverOp, telemetry,
                ShootingPresets.DIST_FAR, ShootingPresets.GOAL_2_HEIGHT, ShootingPresets.GOAL_2_PIPELINE);


        // --- DRIVER CONTROLS ---
        
        // Aligners (A, B, X) + LB Modifier
        driverOp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> {
                    if (driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) schedule(goal2_Close);
                    else schedule(goal1_Close);
                });

        driverOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> {
                    if (driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) schedule(goal2_Mid);
                    else schedule(goal1_Mid);
                });

        driverOp.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> {
                    if (driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) schedule(goal2_Far);
                    else schedule(goal1_Far);
                });

        // Feeders (Triggers)
        if (shooter != null) {
            new Trigger(() -> driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1)
                    .whileActiveContinuous(new InstantCommand(shooter::feedLeft))
                    .whenInactive(new InstantCommand(shooter::stopLeft));

            new Trigger(() -> driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                    .whileActiveContinuous(new InstantCommand(shooter::feedRight))
                    .whenInactive(new InstantCommand(shooter::stopRight));
        }

        // --- OPERATOR CONTROLS ---

        // Flywheel Spin Up (A = Shoot Far, B = Stop)
        if (shooter != null) {
            operatorOp.getGamepadButton(GamepadKeys.Button.A)
                    .whenPressed(new InstantCommand(shooter::shoot_far));
            
            operatorOp.getGamepadButton(GamepadKeys.Button.B)
                    .whenPressed(new InstantCommand(shooter::stopFlywheels));
        }

        // Sort Arm (Bumpers)
        if (intake != null) {
            operatorOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                    .whenPressed(new InstantCommand(() -> intake.sort(false))); // Left

            operatorOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                    .whenPressed(new InstantCommand(() -> intake.sort(true))); // Right
            
            // Optional: Intake Motor Control for Operator (Triggers)
            new Trigger(() -> operatorOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                    .whileActiveContinuous(new InstantCommand(intake::turnOnIntake))
                    .whenInactive(new InstantCommand(intake::turnOffIntake));
        }
    }
}

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

import commands.OdoAlignCommand;
import commands.TeleOpDriveCommand;
import subsystems.IntakeSubsystem;
import subsystems.MecanumSubsystem;
import subsystems.ShooterSubsystem;

@TeleOp
public class MainTeleOp extends CommandOpMode {

    private Limelight3A limelight; // Kept for future use, not used for align now
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
        drive = new MecanumSubsystem(hardwareMap, new Pose2d(0,0,0), true, telemetry);
        
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
            telemetry.addData("Warning", e);
        }

        // 2. Initialize Gamepads
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx operatorOp = new GamepadEx(gamepad2);

        // 3. Default Drive
        drive.setDefaultCommand(new TeleOpDriveCommand(drive, driverOp));

        // --- RESET POSE COMMAND ---
        driverOp.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(() -> drive.setPose(0, 0, 0)));


        // --- ALIGNMENT COMMANDS (Odometry Based) ---
        // Only active while button is held down

        // BLUE SIDE (Left) - D-Pad
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whileHeld(new OdoAlignCommand(drive, AutoConstants.BLUE_SCORE_FAR));
                
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(new OdoAlignCommand(drive, AutoConstants.BLUE_SCORE_CLOSE));

        // RED SIDE (Right) - Letters
        driverOp.getGamepadButton(GamepadKeys.Button.Y)
                .whileHeld(new OdoAlignCommand(drive, AutoConstants.RED_SCORE_FAR));

        driverOp.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(new OdoAlignCommand(drive, AutoConstants.RED_SCORE_CLOSE));


        // --- SHOOTER CONTROLS ---
        // Feeders (Triggers)
        if (shooter != null) {
            new Trigger(() -> driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0)
                    .whileActiveContinuous(new InstantCommand(shooter::feedLeft))
                    .whenInactive(new InstantCommand(shooter::stopLeft));

            new Trigger(() -> driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0)
                    .whileActiveContinuous(new InstantCommand(shooter::feedRight))
                    .whenInactive(new InstantCommand(shooter::stopRight));
        }

        // --- OPERATOR CONTROLS ---

        // Flywheel Spin Up (A = Shoot Far, B = Stop)
        if (shooter != null) {
            operatorOp.getGamepadButton(GamepadKeys.Button.A)
                    .whenPressed(new InstantCommand(shooter::shoot_close));
            
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

        if(shooter != null) {
            operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                    .whenPressed(new InstantCommand(() -> shooter.increaseShootFar()));
            operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(new InstantCommand(() -> shooter.decreaseShootFar()));
            operatorOp.getGamepadButton(GamepadKeys.Button.Y)
                    .whenPressed(new InstantCommand(() -> shooter.increaseShootClose()));
            operatorOp.getGamepadButton(GamepadKeys.Button.A)
                    .whenPressed(new InstantCommand(() -> shooter.decreaseShootClose()));
        }

        telemetry.addData("Shooter Far", shooter.getShootFar());
        telemetry.addData("Shooter Close", shooter.getShootClose());
        telemetry.update();
    }
}

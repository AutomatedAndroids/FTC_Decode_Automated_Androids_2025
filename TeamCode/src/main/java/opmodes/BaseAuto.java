package opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.arcrobotics.ftclib.command.Command;
import commands.RoadRunnerActionCommand;
import subsystems.IntakeSubsystem;
import subsystems.MecanumSubsystem;
import subsystems.ShooterSubsystem;

public abstract class BaseAuto extends CommandOpMode {

    protected MecanumSubsystem drive;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;

    @Override
    public void initialize() {
        Pose2d startPose = getStartPose();
        
        // Initialize Drive with Pinpoint support (true)
        // If Pinpoint fails, it falls back to encoders automatically in MecanumSubsystem
        drive = new MecanumSubsystem(hardwareMap, startPose, true, telemetry);

        // Initialize Shooter
        try {
            // Using names from your subsystems or standard guesses
            CRServo leftFeeder = new CRServo(hardwareMap, "leftFeeder");
            CRServo rightFeeder = new CRServo(hardwareMap, "rightFeeder");
            MotorEx shooterMotor = new MotorEx(hardwareMap, "shooter");
            shooter = new ShooterSubsystem(leftFeeder, rightFeeder, shooterMotor, telemetry);
        } catch (Exception e) {
            telemetry.addData("Warning", "Shooter not found/configured");
        }

        // Initialize Intake
        try {
            Motor intakeMotor = new Motor(hardwareMap, "intake");
            Servo sortArm = hardwareMap.get(Servo.class, "sortArm");
            intake = new IntakeSubsystem(intakeMotor, sortArm, telemetry);
        } catch (Exception e) {
             telemetry.addData("Warning", "Intake not found/configured");
        }
        
        buildAuto();
    }
    
    protected abstract Pose2d getStartPose();
    protected abstract void buildAuto();
    
    // Helper to run RR Actions as FTCLib Commands
    protected Command runAction(com.acmerobotics.roadrunner.Action action) {
        return new RoadRunnerActionCommand(action);
    }
}


package opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import subsystems.IntakeSubsystem;
import subsystems.MecanumDriveSubsystem;
import subsystems.ShooterSubsystem;
import subsystems.VisionSubsystem;

/**
 * Base class for autonomous opmodes.
 * 
 * To create a new autonomous opmode:
 * 1. Extend this class
 * 2. Override getStartPose() to return your starting position
 * 3. Override buildAuto() to schedule your autonomous commands
 * 
 * Example:
 * 
 * @Autonomous(name = "My Auto", group = "Autonomous")
 * public class MyAuto extends BaseAuto {
 *     @Override
 *     protected Pose2d getStartPose() {
 *         return new Pose2d(0, 0, 0); // x, y, heading (radians)
 *     }
 * 
 *     @Override
 *     protected void buildAuto() {
 *         schedule(new SequentialCommandGroup(
 *             new SimpleDriveCommand(drive, 0.5, 0, 0, 2.0),
 *             new TurnOnIntakeCommand(intake),
 *             // ... more commands
 *         ));
 *     }
 * }
 */
public abstract class BaseAuto extends CommandOpMode {

    protected MecanumDriveSubsystem drive;
    protected VisionSubsystem vision;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;

    @Override
    public void run() {
        super.run(); // This runs the command scheduler
        telemetry.update(); // This displays telemetry
    }

    @Override
    public void initialize() {
        Pose2d startPose = getStartPose();
        
        // Initialize Vision Subsystem (handles AprilTag and Limelight)
        vision = new VisionSubsystem(hardwareMap, telemetry);

        // Initialize Drive Subsystem (uses FTCLib motors with velocity control)
        drive = MecanumDriveSubsystem.create(
                hardwareMap,
                startPose,
                telemetry
        );

        // Connect Limelight to drive subsystem if vision initialized successfully
        if (vision != null && vision.getLimelight() != null) {
            drive.setVisionProcessors(
                    null, // No webcam - using Limelight
                    vision.getLimelight()
            );
        }

        drive.enableDrive();

        // Initialize Shooter Subsystem
        try {
            CRServo leftFeeder = new CRServo(hardwareMap, "leftFeeder");
            CRServo rightFeeder = new CRServo(hardwareMap, "rightFeeder");
            Servo leftSaftey = hardwareMap.get(Servo.class, "leftSaftey");
            Servo rightSaftey = hardwareMap.get(Servo.class, "rightSaftey");
            MotorEx shooterMotor = new MotorEx(hardwareMap, "shooter");
            shooter = new ShooterSubsystem(leftFeeder, rightFeeder, shooterMotor, leftSaftey, rightSaftey, telemetry);
        } catch (Exception e) {
            telemetry.addData("Warning", "Shooter not found/configured");
            telemetry.update();
        }

        // Initialize Intake Subsystem
        try {
            Motor intakeMotor = new Motor(hardwareMap, "intake");
            Servo sortArm = hardwareMap.get(Servo.class, "sortArm");
            intake = new IntakeSubsystem(intakeMotor, sortArm, telemetry);
        } catch (Exception e) {
            telemetry.addData("Warning", "Intake not found/configured");
            telemetry.update();
        }
        
        // Build and schedule the autonomous sequence
        buildAuto();
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        
        // Cleanup
        if (vision != null) {
            vision.stop();
        }
        reset();
    }
    
    /**
     * Returns the starting pose of the robot for this autonomous.
     * Override this method to specify your starting position.
     * 
     * @return Pose2d with x (inches), y (inches), and heading (radians)
     */
    protected abstract Pose2d getStartPose();
    
    /**
     * Builds and schedules the autonomous command sequence.
     * Override this method to define your autonomous routine.
     * 
     * Use schedule() to add commands or command groups.
     * Example:
     *   schedule(new SequentialCommandGroup(
     *       new SimpleDriveCommand(drive, 0.5, 0, 0, 2.0),
     *       new TurnOnIntakeCommand(intake),
     *       // ... more commands
     *   ));
     */
    protected abstract void buildAuto();
}

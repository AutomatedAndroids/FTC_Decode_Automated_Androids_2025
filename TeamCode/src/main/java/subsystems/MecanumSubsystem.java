package subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder; // IMPORT ADDED
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import drive.MecanumDrive;
import drive.PinpointDrive;

public class MecanumSubsystem extends SubsystemBase {

    private final Telemetry telemetry;
    private MecanumDrive drive;
    private final PIDController alignController;
    private final PIDController distanceController;

    // Tuning
    public static double kP_turn = 0.03;
    public static double kI_turn = 0.0;
    public static double kD_turn = 0.001;

    public static double kP_dist = 0.04;
    public static double kI_dist = 0.0;
    public static double kD_dist = 0.001;

    public MecanumSubsystem(HardwareMap hardwareMap, Pose2d startPose, boolean usePinpoint, Telemetry telemetry) {
        this.telemetry = telemetry;
        if (usePinpoint) {
            try {
                drive = new PinpointDrive(hardwareMap, startPose);
            } catch (Exception e) {
                // Fallback to standard MecanumDrive if Pinpoint fails (e.g. config missing)
                drive = new MecanumDrive(hardwareMap, startPose);
            }
        } else {
            drive = new MecanumDrive(hardwareMap, startPose);
        }
        
        alignController = new PIDController(kP_turn, kI_turn, kD_turn);
        distanceController = new PIDController(kP_dist, kI_dist, kD_dist);
    }

    @Override
    public void periodic() {
        if (drive != null) {
            drive.updatePoseEstimate();
        
            if (telemetry != null) {
                telemetry.addData("Robot Pose X", drive.pose.position.x);
                telemetry.addData("Robot Pose Y", drive.pose.position.y);
                telemetry.addData("Robot Heading", Math.toDegrees(drive.pose.heading.toDouble()));

                if (drive instanceof PinpointDrive) {
                    PinpointDrive ppDrive = (PinpointDrive) drive;
                    if (ppDrive.pinpoint != null) {
                        Pose2D pos = ppDrive.pinpoint.getPosition();
                        telemetry.addData("Pinpoint X (in)", pos.getX(DistanceUnit.INCH));
                        telemetry.addData("Pinpoint Y (in)", pos.getY(DistanceUnit.INCH));
                        telemetry.addData("Pinpoint Heading (deg)", pos.getHeading(AngleUnit.DEGREES));
                    }
                }
            }
        }
    }

    public void drive(double strafe, double forward, double turn) {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, -strafe), -turn));
    }

    public void driveToGoal(double strafe, double forward, double turn, double currentDist, double targetDist, double headingError) {
        double turnOutput = alignController.calculate(headingError, 0);
        
        // NOTE: Standard PID is calculate(measurement, setpoint).
        // Here we use calculate(setpoint, measurement) -> (target - current).
        // If Current > Target (Too Far), Output is Positive.
        // We assume Positive Forward Power moves the robot closer to the target.
        double forwardOutput = distanceController.calculate(targetDist, currentDist);

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(forwardOutput + forward, -strafe),
                turnOutput - turn
        ));
    }

    public boolean driveToPosition(double xTicks, double yTicks, double headingRad) {
        return drive.driveToPosition(xTicks, yTicks, headingRad);
    }

    public void driveToPositionStupid(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode, Pose2d target) {
        drive.driveToPositionStupid(opMode, target);
    }

    public Pose2d getPose() {
        return drive.pose;
    }
    
    public void setPose(double xTicks, double yTicks, double headingDeg) {
        double xInches = xTicks / MecanumDrive.TICKS_PER_INCH;
        double yInches = yTicks / MecanumDrive.TICKS_PER_INCH;
        drive.pose = new Pose2d(xInches, yInches, Math.toRadians(headingDeg));
    }

    // --- THIS METHOD FIXED ---
    // Changed return type from 'ActionBuilder' to 'TrajectoryActionBuilder'
    public TrajectoryActionBuilder actionBuilder(Pose2d pose) {
        return drive.actionBuilder(pose);
    }
}

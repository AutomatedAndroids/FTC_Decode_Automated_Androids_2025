package subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder; // IMPORT ADDED
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import drive.PinpointDrive;

public class MecanumSubsystem extends SubsystemBase {

    private final PinpointDrive drive;
    private final PIDController alignController;
    private final PIDController distanceController;

    // Tuning
    public static double kP_turn = 0.03;
    public static double kI_turn = 0.0;
    public static double kD_turn = 0.001;

    public static double kP_dist = 0.04;
    public static double kI_dist = 0.0;
    public static double kD_dist = 0.001;

    public MecanumSubsystem(HardwareMap hardwareMap, Pose2d startPose) {
        drive = new PinpointDrive(hardwareMap, startPose);
        alignController = new PIDController(kP_turn, kI_turn, kD_turn);
        distanceController = new PIDController(kP_dist, kI_dist, kD_dist);
    }

    @Override
    public void periodic() {
        drive.updatePoseEstimate();
    }

    public void drive(double strafe, double forward, double turn) {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, -strafe), -turn));
    }

    public void driveToGoal(double strafe, double currentDist, double targetDist, double headingError) {
        double turnOutput = alignController.calculate(headingError, 0);
        double forwardOutput = distanceController.calculate(targetDist, currentDist);

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(forwardOutput, -strafe),
                turnOutput
        ));
    }

    // --- THIS METHOD FIXED ---
    // Changed return type from 'ActionBuilder' to 'TrajectoryActionBuilder'
    public TrajectoryActionBuilder actionBuilder(Pose2d pose) {
        return drive.actionBuilder(pose);
    }
}
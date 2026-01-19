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

    // Flag to enable/disable pose estimation updates (default true for accurate localization)
    private boolean enablePoseUpdates = true;

    // Store last drive command values for telemetry
    private double lastStrafe = 0;
    private double lastForward = 0;
    private double lastTurn = 0;

    @Override
    public void periodic() {
        if (drive != null) {
            // Always update pose estimate for accurate localization
            if (enablePoseUpdates) {
                drive.updatePoseEstimate();
            }
        
            // Always show position telemetry
            if (telemetry != null) {
                telemetry.addData("=== ROBOT POSITION ===", "");
                telemetry.addData("X (in)", String.format("%.2f", drive.pose.position.x));
                telemetry.addData("Y (in)", String.format("%.2f", drive.pose.position.y));
                telemetry.addData("Heading (deg)", String.format("%.2f", Math.toDegrees(drive.pose.heading.toDouble())));

                // Display target and movement information
                if (drive.isProfileActive()) {
                    telemetry.addData("=== MOVING TO TARGET ===", "");
                    Pose2d targetPose = drive.getTargetPose();
                    if (targetPose != null) {
                        telemetry.addData("Target X (in)", String.format("%.2f", targetPose.position.x));
                        telemetry.addData("Target Y (in)", String.format("%.2f", targetPose.position.y));
                        telemetry.addData("Target Heading (deg)", String.format("%.2f", Math.toDegrees(targetPose.heading.toDouble())));
                        telemetry.addData("Distance to Target (in)", String.format("%.2f", drive.getDistanceToTarget()));
                        telemetry.addData("Heading Error (deg)", String.format("%.2f", Math.toDegrees(drive.getHeadingErrorToTarget())));
                    }
                    telemetry.addData("Current Phase", drive.getCurrentPhase().toString());
                }

                if (drive instanceof PinpointDrive) {
                    PinpointDrive ppDrive = (PinpointDrive) drive;
                    if (ppDrive.pinpoint != null) {
                        telemetry.addData("=== PINPOINT ODOMETRY ===", "");
                        Pose2D pos = ppDrive.pinpoint.getPosition();
                        telemetry.addData("Pinpoint X (in)", String.format("%.2f", pos.getX(DistanceUnit.INCH)));
                        telemetry.addData("Pinpoint Y (in)", String.format("%.2f", pos.getY(DistanceUnit.INCH)));
                        telemetry.addData("Pinpoint Heading (deg)", String.format("%.2f", pos.getHeading(AngleUnit.DEGREES)));
                    }
                }
            }
        }
        
        // Drive command telemetry disabled to avoid overriding shooter telemetry
        // Uncomment below if needed for debugging
        /*
        if (telemetry != null) {
            telemetry.addData("=== DRIVE DEBUG ===", "");
            telemetry.addData("Strafe Input", String.format("%.3f", lastStrafe));
            telemetry.addData("Forward Input", String.format("%.3f", lastForward));
            telemetry.addData("Turn Input", String.format("%.3f", lastTurn));
            if (drive != null) {
                telemetry.addData("FL Power", String.format("%.3f", drive.getLastLeftFrontPower()));
                telemetry.addData("FR Power", String.format("%.3f", drive.getLastRightFrontPower()));
                telemetry.addData("BL Power", String.format("%.3f", drive.getLastLeftBackPower()));
                telemetry.addData("BR Power", String.format("%.3f", drive.getLastRightBackPower()));
            }
        }
        */
    }

    public void drive(double strafe, double forward, double turn) {
        // Store values for telemetry
        lastStrafe = strafe;
        lastForward = forward;
        lastTurn = turn;
        
        // Direct blind drive - just pass joystick values straight to motors
        // No pose estimation, no encoder feedback - simple and reliable
        drive.setDrivePowers(forward, strafe, turn);
    }


    public boolean driveToPosition(double xTicks, double yTicks, double headingRad) {
        return drive.driveToPosition(xTicks, yTicks, headingRad);
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

}

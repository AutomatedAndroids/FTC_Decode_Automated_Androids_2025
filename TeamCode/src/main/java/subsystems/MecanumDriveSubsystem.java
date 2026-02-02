package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import Config.DriveConstants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDriveSubsystem extends SubsystemBase {
    private Motor frontLeft, frontRight, backLeft, backRight;
    private GyroEx gyro;
    private AprilTagProcessor webcamAprilTag;
    private Limelight3A limelightApriltag;
    private Telemetry telemetry;
    private Pose2d pose;
    private boolean driveEnabled = false;
    
    // Odometry tracking
    private int lastFrontLeftPos = 0;
    private int lastFrontRightPos = 0;
    private int lastBackLeftPos = 0;
    private int lastBackRightPos = 0;
    private double lastHeading = 0;
    private boolean odometryInitialized = false;
    
    public double ACHIEVABLE_MAX_DISTANCE_PER_SECOND;

    public MecanumDriveSubsystem(
            Motor frontLeft,
            Motor frontRight,
            Motor backLeft,
            Motor backRight,
            GyroEx gyro,
            AprilTagProcessor webcamAprilTag,
            Limelight3A limelightApriltag,
            Pose2d initialPose,
            Telemetry telemetry) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.gyro = gyro;
        this.webcamAprilTag = webcamAprilTag;
        this.limelightApriltag = limelightApriltag;
        this.pose = initialPose;
        this.telemetry = telemetry;
    }

    public static MecanumDriveSubsystem create(HardwareMap hardwareMap, Pose2d initialPose, Telemetry telemetry) {
        // Initialize motors - using same names as teleOp (fL, fR, bL, bR)
        Motor frontLeft = new Motor(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        Motor frontRight = new Motor(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        Motor backLeft = new Motor(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        Motor backRight = new Motor(hardwareMap, "bR", Motor.GoBILDA.RPM_312);

        frontLeft.setInverted(true);
        backLeft.setInverted(true);

        frontLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.stopAndResetEncoder();
        frontRight.stopAndResetEncoder();
        backLeft.stopAndResetEncoder();
        backRight.stopAndResetEncoder();

        frontLeft.setRunMode(Motor.RunMode.VelocityControl);
        frontRight.setRunMode(Motor.RunMode.VelocityControl);
        backLeft.setRunMode(Motor.RunMode.VelocityControl);
        backRight.setRunMode(Motor.RunMode.VelocityControl);

        frontLeft.setVeloCoefficients(1.2, 0, 0.01);
        frontRight.setVeloCoefficients(1.2, 0, 0.01);
        backLeft.setVeloCoefficients(1.2, 0, 0.01);
        backRight.setVeloCoefficients(1.2, 0, 0.01);

        frontLeft.setFeedforwardCoefficients(0.4, 0.6, 0.5);
        frontRight.setFeedforwardCoefficients(0.4, 0.6, 0.5);
        backLeft.setFeedforwardCoefficients(0.2, 0.6, 0.5);
        backRight.setFeedforwardCoefficients(0.2, 0.6, 0.5);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        frontLeft.encoder.reset();
        frontRight.encoder.reset();
        backLeft.encoder.reset();
        backRight.encoder.reset();
        frontLeft.encoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        frontRight.encoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        backLeft.encoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        backRight.encoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);

        backRight.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize gyro
        GyroEx gyro = new GyroEx() {
            IMU imu = hardwareMap.get(IMU.class, "imu");

            @Override
            public void init() {
                RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                        RevHubOrientationOnRobot.LogoFacingDirection.UP;
                RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
                RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                        logoDirection, usbDirection);

                imu.initialize(new IMU.Parameters(orientationOnRobot));
                imu.resetYaw();
            }

            @Override
            public double getHeading() {
                return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            @Override
            public double getAbsoluteHeading() {
                return 0;
            }

            @Override
            public double[] getAngles() {
                return new double[]{
                        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),
                        imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES),
                        imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES)
                };
            }

            @Override
            public com.arcrobotics.ftclib.geometry.Rotation2d getRotation2d() {
                return new com.arcrobotics.ftclib.geometry.Rotation2d(getHeading());
            }

            @Override
            public void reset() {
                imu.resetYaw();
            }

            @Override
            public void disable() {
            }

            @Override
            public String getDeviceType() {
                return "Internal IMU";
            }
        };

        gyro.init();

        // Get vision processors from VisionSubsystem (will be passed in)
        // For now, we'll accept null and set them later if needed
        AprilTagProcessor webcamAprilTag = null;
        Limelight3A limelightApriltag = null;

        MecanumDriveSubsystem subsystem = new MecanumDriveSubsystem(
                frontLeft, frontRight, backLeft, backRight,
                gyro, webcamAprilTag, limelightApriltag,
                initialPose, telemetry);

        // Calculate achievable max distance per second
        double ACHIEVABLE_MAX_TICKS_PER_SECOND = frontLeft.ACHIEVABLE_MAX_TICKS_PER_SECOND;
        subsystem.ACHIEVABLE_MAX_DISTANCE_PER_SECOND = ACHIEVABLE_MAX_TICKS_PER_SECOND * DriveConstants.DISTANCE_PER_PULSE;

        return subsystem;
    }

    public void setVisionProcessors(AprilTagProcessor webcamAprilTag, Limelight3A limelightApriltag) {
        this.webcamAprilTag = webcamAprilTag;
        this.limelightApriltag = limelightApriltag;
    }

    public void enableDrive() {
        driveEnabled = true;
    }

    public void disableDrive() {
        driveEnabled = false;
        setDrivePowers(0, 0, 0);
    }

    public void setDrivePowers(double forward, double strafe, double turn) {
        if (!driveEnabled) return;

        // Mecanum drive calculations
        double flPower = forward + strafe - turn;
        double frPower = forward - strafe + turn;
        double blPower = forward - strafe - turn;
        double brPower = forward + strafe + turn;

        // Normalize if needed
        double maxPower = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower),
                Math.max(Math.abs(blPower), Math.abs(brPower))));
        if (maxPower > 1.0) {
            flPower /= maxPower;
            frPower /= maxPower;
            blPower /= maxPower;
            brPower /= maxPower;
        }

        frontLeft.set(flPower);
        frontRight.set(frPower);
        backLeft.set(blPower);
        backRight.set(brPower);
    }

    public GyroEx getGyro() {
        return gyro;
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public double getAchievableMaxDistancePerSecond() {
        return ACHIEVABLE_MAX_DISTANCE_PER_SECOND;
    }

    @Override
    public void periodic() {
        super.periodic();
        
        // Update pose estimate using encoders and gyro
        updatePoseEstimate();
        
        // Update pose telemetry
        if (telemetry != null) {
            telemetry.addData("=== ROBOT POSITION ===", "");
            telemetry.addData("X (in)", String.format("%.2f", pose.position.x));
            telemetry.addData("Y (in)", String.format("%.2f", pose.position.y));
            telemetry.addData("Heading (deg)", String.format("%.2f", Math.toDegrees(pose.heading.toDouble())));
            
            // Show gyro heading for reference
            if (gyro != null) {
                telemetry.addData("Gyro Heading (deg)", String.format("%.2f", Math.toDegrees(gyro.getHeading())));
            }
        }
    }
    
    /**
     * Updates the pose estimate using encoder deltas and gyro heading.
     * This is a simple odometry implementation - for more accurate tracking,
     * consider using RoadRunner's localizer or a more sophisticated approach.
     */
    private void updatePoseEstimate() {
        if (gyro == null) return;
        
        // Get current encoder positions (in encoder ticks)
        int currentFrontLeftPos = frontLeft.encoder.getPosition();
        int currentFrontRightPos = frontRight.encoder.getPosition();
        int currentBackLeftPos = backLeft.encoder.getPosition();
        int currentBackRightPos = backRight.encoder.getPosition();
        
        // Get current heading from gyro
        double currentHeading = gyro.getHeading();
        
        if (!odometryInitialized) {
            // Initialize odometry
            lastFrontLeftPos = currentFrontLeftPos;
            lastFrontRightPos = currentFrontRightPos;
            lastBackLeftPos = currentBackLeftPos;
            lastBackRightPos = currentBackRightPos;
            lastHeading = currentHeading;
            odometryInitialized = true;
            return;
        }
        
        // Calculate encoder deltas (in encoder ticks)
        int deltaFrontLeft = currentFrontLeftPos - lastFrontLeftPos;
        int deltaFrontRight = currentFrontRightPos - lastFrontRightPos;
        int deltaBackLeft = currentBackLeftPos - lastBackLeftPos;
        int deltaBackRight = currentBackRightPos - lastBackRightPos;
        
        // Convert ticks to distance (meters), then to inches
        // DISTANCE_PER_PULSE is in meters per tick
        double metersPerTick = DriveConstants.DISTANCE_PER_PULSE;
        double inchesPerTick = metersPerTick * 39.37;
        
        // Average forward/backward movement
        // Forward = average of all wheels
        double forwardDelta = ((deltaFrontLeft + deltaFrontRight + deltaBackLeft + deltaBackRight) / 4.0) 
                * inchesPerTick;
        
        // Average strafe movement (left/right)
        // For mecanum: strafe = (leftFront - rightFront - leftBack + rightBack) / 4
        // Note: Adjust signs based on your robot's configuration
        double strafeDelta = ((deltaFrontLeft - deltaFrontRight - deltaBackLeft + deltaBackRight) / 4.0)
                * inchesPerTick;
        
        // Calculate heading change
        double headingDelta = currentHeading - lastHeading;
        // Normalize heading delta to [-pi, pi]
        while (headingDelta > Math.PI) headingDelta -= 2 * Math.PI;
        while (headingDelta <= -Math.PI) headingDelta += 2 * Math.PI;
        
        // Update pose in world frame
        // Rotate robot-frame deltas to world frame
        double cosHeading = Math.cos(pose.heading.toDouble());
        double sinHeading = Math.sin(pose.heading.toDouble());
        
        double worldXDelta = forwardDelta * cosHeading - strafeDelta * sinHeading;
        double worldYDelta = forwardDelta * sinHeading + strafeDelta * cosHeading;
        
        // Update pose
        pose = new Pose2d(
                pose.position.x + worldXDelta,
                pose.position.y + worldYDelta,
                currentHeading
        );
        
        // Store current values for next iteration
        lastFrontLeftPos = currentFrontLeftPos;
        lastFrontRightPos = currentFrontRightPos;
        lastBackLeftPos = currentBackLeftPos;
        lastBackRightPos = currentBackRightPos;
        lastHeading = currentHeading;
    }
}

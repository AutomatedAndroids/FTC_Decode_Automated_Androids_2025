package drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

public class PinpointDrive extends MecanumDrive {
    public static class Params {
        public String pinpointDeviceName = "odo"; // TODO: CHECK CONFIG NAME
        public double xOffset = -3.3071; // TODO: MEASURE (Forward is positive)
        public double yOffset = -6.6142; // TODO: MEASURE (Left is positive)

        // 19.89... for 4-Bar Pods, 13.26... for Swingarm
        public double encoderResolution = 19.89436789;

        public GoBildaPinpointDriver.EncoderDirection xDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }

    public static Params PARAMS = new Params();
    public GoBildaPinpointDriver pinpoint;
    private Pose2d lastPinpointPose;

    public PinpointDrive(HardwareMap hardwareMap, Pose2d startPose) {
        super(hardwareMap, startPose);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PARAMS.pinpointDeviceName);
        pinpoint.setOffsets(
                DistanceUnit.MM.fromInches(PARAMS.xOffset),
                DistanceUnit.MM.fromInches(PARAMS.yOffset)
        );
        pinpoint.setEncoderResolution(PARAMS.encoderResolution);
        pinpoint.setEncoderDirections(PARAMS.xDirection, PARAMS.yDirection);

        pinpoint.resetPosAndIMU();
        try { Thread.sleep(300); } catch (InterruptedException e) { throw new RuntimeException(e); }

        pinpoint.setPosition(rrPoseToFtcPose(startPose));
        this.pose = startPose;
        this.lastPinpointPose = startPose;
    }

    @Override
    public PoseVelocity2d updatePoseEstimate() {
        pinpoint.update();
        if (lastPinpointPose != pose) {
            pinpoint.setPosition(rrPoseToFtcPose(pose));
        }
        Pose2D ftcPose = pinpoint.getPosition();
        pose = new Pose2d(
                ftcPose.getX(DistanceUnit.INCH),
                ftcPose.getY(DistanceUnit.INCH),
                ftcPose.getHeading(AngleUnit.RADIANS)
        );
        lastPinpointPose = pose;

        poseHistory.add(pose);
        while (poseHistory.size() > 100) { poseHistory.removeFirst(); }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));

        Pose2D ftcVel = pinpoint.getVelocity();
        return new PoseVelocity2d(
                new Vector2d(ftcVel.getX(DistanceUnit.INCH), ftcVel.getY(DistanceUnit.INCH)),
                ftcVel.getHeading(AngleUnit.RADIANS)
        );
    }

    private Pose2D rrPoseToFtcPose(Pose2d rrPose) {
        return new Pose2D(DistanceUnit.INCH, rrPose.position.x, rrPose.position.y, AngleUnit.RADIANS, rrPose.heading.toDouble());
    }
}
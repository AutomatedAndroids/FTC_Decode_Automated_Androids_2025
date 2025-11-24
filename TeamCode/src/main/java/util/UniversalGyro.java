package util;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class UniversalGyro extends GyroEx {

    private final IMU imu;

    /**
     * @param hardwareMap The hardware map
     * @param name        The name of the IMU in the config (usually "imu")
     */
    public UniversalGyro(HardwareMap hardwareMap, String name) {
        imu = hardwareMap.get(IMU.class, name);
    }

    @Override
    public void init() {
        // Initialize the IMU with default REV Hub parameters.
        // Check your logo/usb direction!
        // Default here is: Logo UP, USB FORWARD.
        // If your hub is mounted differently, change these directions.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public double getHeading() {
        // FTCLib expects heading in degrees, usually 0 to 360 or -180 to 180
        // The SDK returns -180 to 180
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    @Override
    public double getAbsoluteHeading() {
        return getHeading();
    }

    @Override
    public double[] getAngles() {
        // Returns {Heading, Pitch, Roll}
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return new double[]{
                angles.getYaw(AngleUnit.DEGREES),
                angles.getPitch(AngleUnit.DEGREES),
                angles.getRoll(AngleUnit.DEGREES)
        };
    }

    @Override
    public Rotation2d getRotation2d() {
        return null;
    }

    @Override
    public void reset() {
        imu.resetYaw();
    }

    // Required by GyroEx but usually unused for basic driving
    public void stop() {
    }

    // Helper to let you change orientation easily if needed
    public void init(RevHubOrientationOnRobot.LogoFacingDirection logo, RevHubOrientationOnRobot.UsbFacingDirection usb) {
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void disable() {

    }

    @Override
    public String getDeviceType() {
        return "";
    }
}

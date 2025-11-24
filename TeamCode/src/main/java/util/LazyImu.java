package util;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class LazyImu {
    private IMU imu;
    private final HardwareMap hardwareMap;
    private final String name;
    private final RevHubOrientationOnRobot orientation;

    public LazyImu(HardwareMap hardwareMap, String name, RevHubOrientationOnRobot orientation) {
        this.hardwareMap = hardwareMap;
        this.name = name;
        this.orientation = orientation;
    }

    public IMU get() {
        if (imu == null) {
            imu = hardwareMap.get(IMU.class, name);
            imu.initialize(new IMU.Parameters(orientation));
        }
        return imu;
    }
}
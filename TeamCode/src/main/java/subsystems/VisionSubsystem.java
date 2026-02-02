package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Vision subsystem that uses Limelight for AprilTag detection.
 * Limelight has built-in AprilTag detection, so no webcam/VisionPortal needed.
 */
public class VisionSubsystem extends SubsystemBase {
    private Limelight3A limelight;
    private Telemetry telemetry;

    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        initLimelight(hardwareMap);
    }

    private void initLimelight(HardwareMap hardwareMap) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception e) {
            limelight = null;
            if (telemetry != null) {
                telemetry.addLine("Lost Limelight");
                telemetry.update();
            }
            return;
        }

        // Switch to pipeline 0 (configured for AprilTag detection in Limelight web interface)
        limelight.pipelineSwitch(0);
        
        // Start polling for data
        limelight.start();
    }

    /**
     * Gets the Limelight instance for AprilTag detection.
     * Limelight has built-in AprilTag detection - use getLatestResult() to get detections.
     * 
     * @return Limelight3A instance, or null if not available
     */
    public Limelight3A getLimelight() {
        return limelight;
    }

    /**
     * @deprecated Use getLimelight() instead. Limelight handles AprilTag detection internally.
     */
    @Deprecated
    public Limelight3A getLimelightApriltag() {
        return limelight;
    }

    /**
     * @deprecated Webcam not used - Limelight handles vision.
     */
    @Deprecated
    public Object getWebcamAprilTag() {
        return null;
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}

package Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

public class ApriltagsFieldData {
    // Example AprilTag metadata - adjust IDs and poses based on your field
    // AprilTagMetadata constructor: (int id, String name, double height, DistanceUnit)
    // Height is the distance from the floor to the center of the tag
    public static final AprilTagMetadata tag_2 = new AprilTagMetadata(
            2,
            "Tag 2",
            0.0,  // Height in meters (adjust based on your field setup)
            DistanceUnit.METER
    );
    
    public static final AprilTagMetadata tag_42 = new AprilTagMetadata(
            42,
            "Tag 42",
            0.0,  // Height in meters (adjust based on your field setup)
            DistanceUnit.METER
    );
}

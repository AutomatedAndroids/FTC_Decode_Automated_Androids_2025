package opmodes;

import com.acmerobotics.roadrunner.Pose2d;

public class AutoConstants {
    // TODO: Update these coordinates for your specific field setup
    // Assuming RoadRunner coordinates: x forward, y left

    // Starting Poses
    public static final Pose2d BLUE_LEFT_START = new Pose2d(12, 61, Math.toRadians(270));
    public static final Pose2d BLUE_RIGHT_START = new Pose2d(-12, 61, Math.toRadians(270));
    public static final Pose2d RED_LEFT_START = new Pose2d(-12, -61, Math.toRadians(90));
    public static final Pose2d RED_RIGHT_START = new Pose2d(12, -61, Math.toRadians(90));

    // Parking Spots
    public static final Pose2d BLUE_PARK = new Pose2d(48, 60, Math.toRadians(270));
    public static final Pose2d RED_PARK = new Pose2d(48, -60, Math.toRadians(90));

    // Shooting Positions
    public static final Pose2d BLUE_SHOOT_POS = new Pose2d(0, 36, Math.toRadians(270));
    public static final Pose2d RED_SHOOT_POS = new Pose2d(0, -36, Math.toRadians(90));
}


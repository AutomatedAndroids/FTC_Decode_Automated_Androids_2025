package util;

public class ShootingPresets {

    // --- GOAL 1 SETTINGS (e.g., Red Basket) ---
    // If using AprilTags, this is the Tag ID. If using color pipelines, this is the pipeline number.
    public static final int GOAL_1_PIPELINE = 0;
    public static final double GOAL_1_HEIGHT = 20.0; // Inches from floor to target center

    // --- GOAL 2 SETTINGS (e.g., Blue Basket) ---
    public static final int GOAL_2_PIPELINE = 1;
    public static final double GOAL_2_HEIGHT = 20.0; // Inches

    // --- DISTANCES (Inches) ---
    public static final double DIST_CLOSE = 12.0;
    public static final double DIST_MID   = 24.0;
    public static final double DIST_FAR   = 36.0;

    // --- CAMERA CONSTANTS (Measure these!) ---
    public static final double CAMERA_HEIGHT = 7.5; // Height of Limelight from floor
    public static final double CAMERA_ANGLE  = 15.0; // Mounting angle in degrees
    public static final double CAMERA_OFFSET = 6.0; // Distance from center of rotation (forward is positive)
}
package opmodes;

import com.acmerobotics.roadrunner.Pose2d;

public class AutoConstants {
    //parking spots
    public static final Pose2d BLUE_BACK_START = new Pose2d(-23.5, 0, Math.toRadians(0));
    public static final Pose2d BLUE_GOAL_START = new Pose2d(-35, 50, Math.toRadians(-45));

    public static final Pose2d BLUE_PARK = new Pose2d(-32.5, 26, Math.toRadians(0));

    public static final Pose2d RED_BACK_START = new Pose2d(23.5, 0, Math.toRadians(0));
    public static final Pose2d RED_GOAL_START = new Pose2d(35, 50, Math.toRadians(-45));

    public static final Pose2d RED_PARK = new Pose2d(32.5, 26, Math.toRadians(0));


    //scoring spots
    public static final Pose2d BLUE_APEX = new Pose2d(0, 70.5, Math.toRadians(45));
    public static final Pose2d RED_APEX = new Pose2d(0, 70.5, Math.toRadians(-45));

    public static final Pose2d RED_SCORE_CLOSE = new Pose2d(12, 82.5, Math.toRadians(-45));
    public static final Pose2d BLUE_SCORE_CLOSE = new Pose2d(-12, 82.5, Math.toRadians(45));

    public static final Pose2d RED_SCORE_FAR = new Pose2d(0, 15, Math.toRadians(-29));
    public static final Pose2d BLUE_SCORE_FAR = new Pose2d(0, 15, Math.toRadians(29));
}


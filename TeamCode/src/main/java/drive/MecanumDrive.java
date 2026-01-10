package drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public class MecanumDrive {
    public static class Params {
        // Drive Constants
        public double inPerTick = 1;
        public double lateralInPerTick = 1;
        public double trackWidthTicks = 0;

        // Feedforward
        public double kS = 0;
        public double kV = 0;
        public double kA = 0;

        // Path Following Gains
        public double axialGain = 0.0;
        public double lateralGain = 0.0;
        public double headingGain = 0.0;

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0;

        // Constraints
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;
        public double maxAngVel = Math.PI; // 180 deg/s
        public double maxAngAccel = Math.PI;
    }

    public static Params PARAMS = new Params();

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;

    public final Localizer localizer;

    public Pose2d pose;
    public final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    // --- INTERFACE FOR LOCALIZER ---
    public interface Localizer {
        Twist2dDual<Time> update();
    }

    public class DriveLocalizer implements Localizer {
        public final Encoder leftFront, leftBack, rightBack, rightFront;

        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private boolean initialized;

        public DriveLocalizer() {
            leftFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftFront));
            leftBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftBack));
            rightBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightBack));
            rightFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightFront));
        }

        @Override
        public Twist2dDual<Time> update() {
            com.acmerobotics.roadrunner.ftc.PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            com.acmerobotics.roadrunner.ftc.PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
            com.acmerobotics.roadrunner.ftc.PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
            com.acmerobotics.roadrunner.ftc.PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

            if (!initialized) {
                initialized = true;

                lastLeftFrontPos = leftFrontPosVel.position;
                lastLeftBackPos = leftBackPosVel.position;
                lastRightBackPos = rightBackPosVel.position;
                lastRightFrontPos = rightFrontPosVel.position;

                return new Twist2dDual<>(
                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                        DualNum.constant(0.0, 2)
                );
            }

            int leftFrontPosDelta = leftFrontPosVel.position - lastLeftFrontPos;
            int leftBackPosDelta = leftBackPosVel.position - lastLeftBackPos;
            int rightBackPosDelta = rightBackPosVel.position - lastRightBackPos;
            int rightFrontPosDelta = rightFrontPosVel.position - lastRightFrontPos;

            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (double) leftFrontPosDelta * PARAMS.inPerTick,
                            (double) leftFrontPosVel.velocity * PARAMS.inPerTick,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (double) leftBackPosDelta * PARAMS.inPerTick,
                            (double) leftBackPosVel.velocity * PARAMS.inPerTick,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (double) rightBackPosDelta * PARAMS.inPerTick,
                            (double) rightBackPosVel.velocity * PARAMS.inPerTick,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (double) rightFrontPosDelta * PARAMS.inPerTick,
                            (double) rightFrontPosVel.velocity * PARAMS.inPerTick,
                    }).times(PARAMS.inPerTick)
            ));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            return twist;
        }
    }

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        this.pose = pose;

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: MAKE SURE THESE NAMES MATCH YOUR CONFIG ON THE ROBOT
        leftFront = hardwareMap.get(DcMotorEx.class, "fL");
        leftBack = hardwareMap.get(DcMotorEx.class, "bL");
        rightBack = hardwareMap.get(DcMotorEx.class, "bR");
        rightFront = hardwareMap.get(DcMotorEx.class, "fR");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: REVERSE MOTORS HERE IF NEEDED
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new DriveLocalizer();
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        // Direct Mecanum Math for "Stupid Drive"
        // Avoids RoadRunner Kinematics complexity
        
        double y = powers.linearVel.x; // Forward
        double x = powers.linearVel.y; // Strafe
        double rx = powers.angVel;  // Turn

        // Denominator ensures we don't exceed motor max while keeping ratios
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // Turn Left (positive rx) -> Left wheels reverse, Right wheels forward
        double fl = (y + x - rx) / denominator;
        double bl = (y - x - rx) / denominator; // Back Left
        double fr = (y - x + rx) / denominator; // Front Right
        double br = (y + x + rx) / denominator; // Back Right

        // If you see it going wrong direction, FLIP the signs above or REVERSE motors below
        
        leftFront.setPower(fl);
        leftBack.setPower(bl);
        rightFront.setPower(fr);
        rightBack.setPower(br);
    }

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());
        
        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }
        
        estimatedPoseWriter.write(new PoseMessage(pose));
        
        return twist.velocity().value();
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public final class TrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public TrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                TrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    // --- NEW PID POSITION CONTROL ---

    // --- TIME-BASED PROFILE DRIVE IMPLEMENTATION ---

    public static double TICKS_PER_INCH = 505.31; // Approximate for GoBilda Pinpoint (19.89 ticks/mm)

    public static double DRIVE_SPEED = 0.5;
    public static double CORRECTION_SPEED = 0.2;
    public static double TOLERANCE = 1.0; // inches
    
    // Motion profile constraints
    public static double MAX_LINEAR_VEL = 20.0; // inches per second
    public static double MAX_LINEAR_ACCEL = 30.0; // inches per second^2
    public static double MAX_ANG_VEL = Math.PI; // radians per second
    public static double MAX_ANG_ACCEL = Math.PI * 2; // radians per second^2

    // Profile state tracking
    private static class MotionProfile {
        double distance;
        double maxVel;
        double maxAccel;
        double duration;
        double accelTime;
        double cruiseTime;
        boolean triangular; // true if we never reach maxVel
        
        MotionProfile(double dist, double maxV, double maxA) {
            distance = Math.abs(dist);
            maxVel = maxV;
            maxAccel = maxA;
            
            if (distance < 1e-6) {
                // Zero distance
                triangular = true;
                accelTime = 0;
                cruiseTime = 0;
                duration = 0;
                return;
            }
            
            // Calculate time to reach max velocity
            double timeToMaxVel = maxVel / maxAccel;
            // Distance to reach max velocity
            double distToMaxVel = 0.5 * maxAccel * timeToMaxVel * timeToMaxVel;
            
            if (distToMaxVel * 2 >= distance) {
                // Triangular profile - never reach maxVel
                triangular = true;
                accelTime = Math.sqrt(distance / maxAccel);
                cruiseTime = 0;
                duration = accelTime * 2;
            } else {
                // Trapezoidal profile
                triangular = false;
                accelTime = timeToMaxVel;
                double cruiseDist = distance - (distToMaxVel * 2);
                cruiseTime = cruiseDist / maxVel;
                duration = accelTime * 2 + cruiseTime;
            }
        }
        
        // Get target velocity at time t
        double getVelocity(double t) {
            if (t < 0) return 0;
            if (t >= duration) return 0;
            
            if (triangular) {
                if (t < accelTime) {
                    return maxAccel * t;
                } else {
                    return maxAccel * (duration - t);
                }
            } else {
                if (t < accelTime) {
                    return maxAccel * t;
                } else if (t < accelTime + cruiseTime) {
                    return maxVel;
                } else {
                    return maxAccel * (duration - t);
                }
            }
        }
        
        // Get target position at time t
        double getPosition(double t) {
            if (t < 0) return 0;
            if (t >= duration) return distance;
            
            if (triangular) {
                if (t < accelTime) {
                    return 0.5 * maxAccel * t * t;
                } else {
                    double decelStartPos = 0.5 * maxAccel * accelTime * accelTime;
                    double decelTime = t - accelTime;
                    double decelVel = maxAccel * accelTime;
                    return decelStartPos + decelVel * decelTime - 0.5 * maxAccel * decelTime * decelTime;
                }
            } else {
                if (t < accelTime) {
                    return 0.5 * maxAccel * t * t;
                } else if (t < accelTime + cruiseTime) {
                    double accelDist = 0.5 * maxAccel * accelTime * accelTime;
                    return accelDist + maxVel * (t - accelTime);
                } else {
                    double accelDist = 0.5 * maxAccel * accelTime * accelTime;
                    double cruiseDist = maxVel * cruiseTime;
                    double decelTime = t - (accelTime + cruiseTime);
                    double decelVel = maxVel;
                    return accelDist + cruiseDist + decelVel * decelTime - 0.5 * maxAccel * decelTime * decelTime;
                }
            }
        }
    }

    // Profile execution state
    private MotionProfile linearProfile = null;
    private MotionProfile angularProfile = null;
    private long profileStartTimeNs = -1;
    private Pose2d profileStartPose = null;
    private Pose2d profileTargetPose = null;
    private double profileStartHeading = 0;
    private boolean profileActive = false;
    
    // Sequential phase tracking
    public enum Phase {
        TURNING_TO_BEARING,    // Turning to face target position
        DRIVING_TO_POSITION,   // Driving forward to position
        TURNING_TO_FINAL       // Adjusting final heading
    }
    private Phase currentPhase = Phase.TURNING_TO_BEARING;
    private static final double HEADING_TOLERANCE = Math.toRadians(5); // When heading is "close enough" to start driving
    
    // Public getters for telemetry
    public Phase getCurrentPhase() {
        return currentPhase;
    }
    
    public Pose2d getTargetPose() {
        return profileTargetPose;
    }
    
    public boolean isProfileActive() {
        return profileActive;
    }
    
    public double getDistanceToTarget() {
        if (profileTargetPose == null) return 0;
        return Math.hypot(profileTargetPose.position.x - pose.position.x, 
                         profileTargetPose.position.y - pose.position.y);
    }
    
    public double getHeadingErrorToTarget() {
        if (profileTargetPose == null) return 0;
        double dx = profileTargetPose.position.x - pose.position.x;
        double dy = profileTargetPose.position.y - pose.position.y;
        if (Math.hypot(dx, dy) < 1e-6) return 0;
        double absBearing = Math.atan2(dy, dx);
        double headingError = absBearing - pose.heading.toDouble();
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError <= -Math.PI) headingError += 2 * Math.PI;
        return headingError;
    }

    /**
     * Non-blocking drive for compatibility.
     * Uses time-based motion profiles and returns true when complete.
     */
    public boolean driveToPosition(double targetXTicks, double targetYTicks, double targetHeadingRad) {
         updatePoseEstimate();
         
         double targetX = targetXTicks / TICKS_PER_INCH;
         double targetY = targetYTicks / TICKS_PER_INCH;
         Pose2d target = new Pose2d(targetX, targetY, targetHeadingRad);
         
         double dist = Math.hypot(targetX - pose.position.x, targetY - pose.position.y);
         
         // Heading check
         double hError = targetHeadingRad - pose.heading.toDouble();
         while (hError > Math.PI) hError -= 2 * Math.PI;
         while (hError <= -Math.PI) hError += 2 * Math.PI;
         
         if (dist < TOLERANCE && Math.abs(hError) < Math.toRadians(5)) {
             setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
             profileActive = false;
             linearProfile = null;
             angularProfile = null;
             return true;
         }
         
         moveTo(target, DRIVE_SPEED);
         return false;
    }
    
    private void moveTo(Pose2d target, double speed) {
        long currentTimeNs = System.nanoTime();
        
        // Calculate distance and heading errors
        double dx = target.position.x - pose.position.x;
        double dy = target.position.y - pose.position.y;
        double dist = Math.hypot(dx, dy);
        
        double absBearing = Math.atan2(dy, dx);
        double headingError = absBearing - pose.heading.toDouble();
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError <= -Math.PI) headingError += 2 * Math.PI;
        
        double finalHeadingError = target.heading.toDouble() - pose.heading.toDouble();
        while (finalHeadingError > Math.PI) finalHeadingError -= 2 * Math.PI;
        while (finalHeadingError <= -Math.PI) finalHeadingError += 2 * Math.PI;
        
        // Check if we've arrived
        if (dist < TOLERANCE && Math.abs(finalHeadingError) < Math.toRadians(5)) {
            setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            profileActive = false;
            linearProfile = null;
            angularProfile = null;
            currentPhase = Phase.TURNING_TO_BEARING; // Reset for next movement
            return;
        }
        
        // Determine current phase based on state
        Phase desiredPhase;
        if (dist < TOLERANCE) {
            // At position, only adjust final heading
            desiredPhase = Phase.TURNING_TO_FINAL;
        } else if (Math.abs(headingError) > HEADING_TOLERANCE) {
            // Not aligned, need to turn first
            desiredPhase = Phase.TURNING_TO_BEARING;
        } else {
            // Aligned and not at position, drive forward
            desiredPhase = Phase.DRIVING_TO_POSITION;
        }
        
        // If phase changed, reset profile
        if (desiredPhase != currentPhase) {
            profileActive = false;
            linearProfile = null;
            angularProfile = null;
            currentPhase = desiredPhase;
        }
        
        // Re-plan if needed
        boolean needReplan = false;
        if (!profileActive || linearProfile == null || angularProfile == null) {
            needReplan = true;
        } else if (profileTargetPose == null || 
                   Math.hypot(target.position.x - profileTargetPose.position.x, 
                             target.position.y - profileTargetPose.position.y) > TOLERANCE * 2 ||
                   Math.abs(target.heading.toDouble() - profileTargetPose.heading.toDouble()) > Math.toRadians(10)) {
            needReplan = true;
        } else {
            double elapsed = (currentTimeNs - profileStartTimeNs) / 1e9;
            double activeProfileDuration = 0;
            if (currentPhase == Phase.TURNING_TO_BEARING || currentPhase == Phase.TURNING_TO_FINAL) {
                activeProfileDuration = angularProfile != null ? angularProfile.duration : 0;
            } else if (currentPhase == Phase.DRIVING_TO_POSITION) {
                activeProfileDuration = linearProfile != null ? linearProfile.duration : 0;
            }
            if (elapsed > activeProfileDuration + 0.5) {
                // Profile expired, re-plan
                needReplan = true;
            }
        }
        
        if (needReplan) {
            // Create new profiles based on current phase
            profileStartPose = pose;
            profileTargetPose = target;
            profileStartTimeNs = currentTimeNs;
            profileStartHeading = pose.heading.toDouble();
            profileActive = true;
            
            double angularMaxVel = MAX_ANG_VEL * speed;
            double angularMaxAccel = MAX_ANG_ACCEL * speed;
            double linearMaxVel = MAX_LINEAR_VEL * speed;
            double linearMaxAccel = MAX_LINEAR_ACCEL * speed;
            
            if (currentPhase == Phase.TURNING_TO_BEARING) {
                // Plan only angular motion
                double angularDist = Math.abs(headingError);
                angularProfile = new MotionProfile(angularDist, angularMaxVel, angularMaxAccel);
                linearProfile = null; // No linear movement during turning
            } else if (currentPhase == Phase.DRIVING_TO_POSITION) {
                // Plan only linear motion
                linearProfile = new MotionProfile(dist, linearMaxVel, linearMaxAccel);
                angularProfile = null; // No angular movement during driving
            } else if (currentPhase == Phase.TURNING_TO_FINAL) {
                // Plan only angular motion for final heading
                double angularDist = Math.abs(finalHeadingError);
                angularProfile = new MotionProfile(angularDist, angularMaxVel, angularMaxAccel);
                linearProfile = null; // No linear movement during final turn
            }
        }
        
        // Execute profile based on current phase
        double elapsed = profileStartTimeNs >= 0 ? (currentTimeNs - profileStartTimeNs) / 1e9 : 0;
        
        if (currentPhase == Phase.TURNING_TO_BEARING) {
            // Turn only - no linear movement
            double angularSign = (headingError > 0) ? 1 : -1;
            double targetAngularVel = angularProfile.getVelocity(elapsed) * angularSign;
            double turnPower = targetAngularVel / MAX_ANG_VEL;
            turnPower = Math.max(-1, Math.min(1, turnPower));
            setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), turnPower));
            
        } else if (currentPhase == Phase.DRIVING_TO_POSITION) {
            // Drive toward target - no turning, but can strafe if needed
            double profileVel = linearProfile.getVelocity(elapsed);
            
            // Convert absolute bearing to robot-relative forward/strafe components
            // Since we're in driving phase, heading should be roughly aligned
            double relativeBearing = absBearing - pose.heading.toDouble();
            
            // Forward/backward component (along robot's forward direction)
            double forwardVel = profileVel * Math.cos(relativeBearing);
            // Strafe component (along robot's left/right direction)
            double strafeVel = profileVel * Math.sin(relativeBearing);
            
            // Convert to drive powers (normalize by max velocity)
            double forwardPower = forwardVel / MAX_LINEAR_VEL;
            double strafePower = strafeVel / MAX_LINEAR_VEL;
            
            // Clamp to [-1, 1]
            forwardPower = Math.max(-1, Math.min(1, forwardPower));
            strafePower = Math.max(-1, Math.min(1, strafePower));
            
            // Drive with forward/strafe but NO turning (turnPower = 0)
            setDrivePowers(new PoseVelocity2d(new Vector2d(forwardPower, strafePower), 0));
            
        } else if (currentPhase == Phase.TURNING_TO_FINAL) {
            // Turn to final heading only - no linear movement
            double angularSign = (finalHeadingError > 0) ? 1 : -1;
            double targetAngularVel = angularProfile.getVelocity(elapsed) * angularSign;
            double turnPower = targetAngularVel / MAX_ANG_VEL;
            turnPower = Math.max(-1, Math.min(1, turnPower));
            setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), turnPower));
        }
    }

    public void driveToPositionStupid(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode, Pose2d target) {
        while (opMode.opModeIsActive()) {
            updatePoseEstimate();
            if (driveToPosition(target.position.x * TICKS_PER_INCH, target.position.y * TICKS_PER_INCH, target.heading.toDouble())) {
                break;
            }
        }
    }
}

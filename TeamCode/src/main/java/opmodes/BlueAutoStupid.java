package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import drive.GoBildaPinpointDriver;
import opmodes.AutoConstants;
@Autonomous(name = "Blue Auto Stupid (DEBUG)")
public class BlueAutoStupid extends LinearOpMode {

    // Motors
    private DcMotor fl, fr, bl, br;
    private DcMotor shooterMotor;
    private CRServo leftFeeder, rightFeeder;

    // Odometry
    private GoBildaPinpointDriver odo;

    // Constants (Copied from PinpointDrive and AutoConstants)
    // Adjust these if the robot moves weirdly!
    double PINPOINT_X_OFFSET = -3.3071; // inches
    double PINPOINT_Y_OFFSET = -6.6142; // inches


    @Override
    public void runOpMode() throws InterruptedException {
        // ... (Hardware Init) ...
        fl = hardwareMap.get(DcMotor.class, "fL");
        fr = hardwareMap.get(DcMotor.class, "fR");
        bl = hardwareMap.get(DcMotor.class, "bL");
        br = hardwareMap.get(DcMotor.class, "bR");

        // FIX: Match Motor Directions to MecanumDrive (Right side reversed)
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        // Good practice for Auto
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        // ... offsets ...
        // RESET TO 0,0,0. Robot thinks it is at Origin relative to ITSELF.
        odo.resetPosAndIMU();

        telemetry.addLine("Initialized.");
        telemetry.addLine("WAITING FOR START.");
        telemetry.addData("Absolute Start", "%.1f, %.1f", AutoConstants.BLUE_BACK_START.position.x, AutoConstants.BLUE_BACK_START.position.y);
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            goToPosition(AutoConstants.BLUE_PARK, AutoConstants.BLUE_BACK_START, 0.3, 2.0);
            goToPosition(AutoConstants.BLUE_APEX, AutoConstants.BLUE_BACK_START, 0.3, 3.0);
        }
    }

    /**
     * extremely explicit "Stupid" drive function.
     * Calculates RELATIVE target from ABSOLUTE coordinates.
     */
    private void goToPosition(Pose2d absoluteTarget, Pose2d absoluteStart, double maxSpeed, double toleranceInches) {
        // Calculate Relative Target (Where to go from 0,0)
        double targetX = absoluteTarget.position.x - absoluteStart.position.x;
        double targetY = absoluteTarget.position.y - absoluteStart.position.y;
        double targetH_deg = Math.toDegrees(absoluteTarget.heading.toDouble() - absoluteStart.heading.toDouble());

        ElapsedTime runtime = new ElapsedTime();
        
        while (opModeIsActive()) {
            // Update Odo
            odo.update();
            Pose2D pos = odo.getPosition();
            double curX = pos.getX(DistanceUnit.INCH);
            double curY = pos.getY(DistanceUnit.INCH);
            double curH = pos.getHeading(AngleUnit.DEGREES);

            // Calculate Errors
            double xError = targetX - curX;
            double yError = targetY - curY;
            
            // Distance Error
            double distError = Math.hypot(xError, yError);
            
            // Heading Error
            double hError = targetH_deg - curH;
            while (hError > 180) hError -= 360;
            while (hError <= -180) hError += 360;

            // --- TELEMETRY ---
            telemetry.addData(">>> TARGET", "X:%.1f Y:%.1f H:%.1f", targetX, targetY, targetH_deg);
            telemetry.addData(">>> CURRENT", "X:%.1f Y:%.1f H:%.1f", curX, curY, curH);
            telemetry.addData(">>> ERROR", "Dist:%.2f H_Err:%.1f", distError, hError);
            
            // Check Exit Condition
            if (distError < toleranceInches && Math.abs(hError) < 5) {
                telemetry.addLine("TARGET REACHED!");
                telemetry.update();
                break;
            }

            // --- CONTROL LOGIC ---
            
            // 1. Calculate Field Errors
            double xErrorVal = xError;
            double yErrorVal = yError;
            double hErrorVal = hError;

            // 2. Convert to Robot Centric Errors (How far Forward/Strafe do we need to go?)
            double rad = Math.toRadians(-curH);
            double forwardError = xErrorVal * Math.cos(rad) - yErrorVal * Math.sin(rad);
            double strafeError = xErrorVal * Math.sin(rad) + yErrorVal * Math.cos(rad);

            double forward = 0;
            double strafe = 0;
            double turn = 0;

            // 3. SEQUENCING (Robot Centric)
            String movementStatus = "IDLE";
            
            // Tuning Constants
            double TURN_TOLERANCE = 5.0; // degrees
            double MOVE_TOLERANCE = 2.0; // inches
            double MIN_POWER = 0.15;     // Minimum power to move

            if (Math.abs(hErrorVal) > TURN_TOLERANCE) {
                // PHASE 1: TURN
                movementStatus = (hErrorVal > 0) ? "TURNING LEFT" : "TURNING RIGHT";
                turn = hErrorVal * 0.03; 
                // Add min power to ensure it moves
                if (Math.abs(turn) < MIN_POWER) turn = Math.signum(turn) * MIN_POWER;
                
            } else if (Math.abs(forwardError) > MOVE_TOLERANCE) {
                // PHASE 2: DRIVE FORWARD/BACK
                movementStatus = (forwardError > 0) ? "DRIVING FORWARD" : "DRIVING BACK";
                forward = forwardError * 0.05;
                if (Math.abs(forward) < MIN_POWER) forward = Math.signum(forward) * MIN_POWER;

            } else if (Math.abs(strafeError) > MOVE_TOLERANCE) {
                // PHASE 3: STRAFE
                movementStatus = (strafeError > 0) ? "STRAFING LEFT" : "STRAFING RIGHT";
                strafe = strafeError * 0.06; // Strafe usually needs more power
                if (Math.abs(strafe) < MIN_POWER) strafe = Math.signum(strafe) * MIN_POWER;
                
            } else {
                movementStatus = "HOLDING";
                // Optional: Small P hold to keep position
            }

            // 4. Limit Speed
            if (Math.abs(forward) > maxSpeed) forward = Math.signum(forward) * maxSpeed;
            if (Math.abs(strafe) > maxSpeed) strafe = Math.signum(strafe) * maxSpeed;
            if (Math.abs(turn) > maxSpeed) turn = Math.signum(turn) * maxSpeed;

            telemetry.addData("MOVING", movementStatus);

            // 5. Mecanum Mix (Matches TeleOp)
            double flP = forward - strafe - turn;
            double blP = forward + strafe - turn;
            double frP = forward + strafe + turn;
            double brP = forward - strafe + turn;

            // Normalize Motor Powers
            double max = Math.max(Math.abs(flP), Math.max(Math.abs(blP), Math.max(Math.abs(frP), Math.abs(brP))));
            if (max > 1.0) {
                flP /= max;
                blP /= max;
                frP /= max;
                brP /= max;
            }

            fl.setPower(flP);
            bl.setPower(blP);
            fr.setPower(frP);
            br.setPower(brP);

            telemetry.addData("Motors", "FL:%.2f FR:%.2f BL:%.2f BR:%.2f", flP, frP, blP, brP);
            telemetry.update();
        }
        stopMotors();
    }
    
    private void stopMotors() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}


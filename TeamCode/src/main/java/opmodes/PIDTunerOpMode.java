package opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import drive.MecanumDrive;
import drive.PinpointDrive;

@TeleOp(name = "PID Tuner", group = "Tuning")
public class PIDTunerOpMode extends LinearOpMode {

    private MecanumDrive drive;

    // Tuning state
    private enum State {
        MANUAL,
        AUTO_TO_PARK,
        AUTO_TO_START
    }

    private State currentState = State.MANUAL;
    
    // Selection state for tuning
    private enum Variable {
        SPEED,
        CORRECTION_SPEED,
        TOLERANCE
    }

    private Variable selectedVariable = Variable.SPEED;

    // Multiplier for adjustments
    private double tuningMultiplier = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drive with 0,0,0
        try {
            drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
            telemetry.addLine("Using Pinpoint Drive");
        } catch (Exception e) {
            drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            telemetry.addLine("Using Standard Mecanum Drive (Pinpoint failed)");
        }

        telemetry.addLine("Ready to tune Simple Drive.");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Start/Back: Switch States (Manual/Auto)");
        telemetry.addLine("  A: Go to Blue Park (in Auto)");
        telemetry.addLine("  B: Go to Start (in Auto)");
        telemetry.addLine("  D-Pad: Adjust Value");
        telemetry.addLine("  X: Change Variable (Speed -> Corr -> Tol)");
        telemetry.addLine("  LB/RB: Change Tuning Multiplier");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update pose estimate
            drive.updatePoseEstimate();

            handleInput();

            switch (currentState) {
                case MANUAL:
                    driveNormal();
                    break;
                case AUTO_TO_PARK:
                    runAutoToPark();
                    // Automatically return to manual after blocking move finishes
                    currentState = State.MANUAL; 
                    break;
                case AUTO_TO_START:
                    runAutoToStart();
                    currentState = State.MANUAL;
                    break;
            }

            displayTelemetry();
        }
    }

    private void handleInput() {
        // Trigger Auto Moves
        if (gamepad1.a) {
             currentState = State.AUTO_TO_PARK;
        } else if (gamepad1.b) {
             currentState = State.AUTO_TO_START;
        }

        // Variable Selection
        if (gamepad1.x && !previousX) {
            switch (selectedVariable) {
                case SPEED: selectedVariable = Variable.CORRECTION_SPEED; break;
                case CORRECTION_SPEED: selectedVariable = Variable.TOLERANCE; break;
                case TOLERANCE: selectedVariable = Variable.SPEED; break;
            }
        }
        previousX = gamepad1.x;

        // Multiplier
        if (gamepad1.right_bumper && !previousRB) {
            tuningMultiplier *= 10;
        }
        previousRB = gamepad1.right_bumper;
        
        if (gamepad1.left_bumper && !previousLB) {
            tuningMultiplier /= 10;
        }
        previousLB = gamepad1.left_bumper;

        // Adjust Values
        boolean up = gamepad1.dpad_up && !previousUp;
        boolean down = gamepad1.dpad_down && !previousDown;
        
        if (up || down) {
            adjustVariable(up ? 1 : -1);
        }
        
        previousUp = gamepad1.dpad_up;
        previousDown = gamepad1.dpad_down;
    }

    // Input state tracking
    boolean previousX, previousRB, previousLB, previousUp, previousDown;

    private void adjustVariable(int direction) {
        double increment = 0.01 * tuningMultiplier; 
        
        switch (selectedVariable) {
            case SPEED: 
                MecanumDrive.DRIVE_SPEED += direction * increment;
                if(MecanumDrive.DRIVE_SPEED < 0) MecanumDrive.DRIVE_SPEED = 0;
                break;
            case CORRECTION_SPEED: 
                MecanumDrive.CORRECTION_SPEED += direction * increment; 
                if(MecanumDrive.CORRECTION_SPEED < 0) MecanumDrive.CORRECTION_SPEED = 0;
                break;
            case TOLERANCE: 
                MecanumDrive.TOLERANCE += direction * increment; 
                if(MecanumDrive.TOLERANCE < 0) MecanumDrive.TOLERANCE = 0;
                break;
        }
    }

    private void driveNormal() {
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));
    }

    private void runAutoToPark() {
        // Target is BLUE_PARK
        // Blocking call
        drive.driveToPositionStupid(this, AutoConstants.BLUE_PARK);
    }

    private void runAutoToStart() {
        // Target is 0,0,0
        drive.driveToPositionStupid(this, new Pose2d(0, 0, 0));
    }

    private void displayTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Selected", selectedVariable);
        telemetry.addData("Multiplier", tuningMultiplier);
        
        telemetry.addData("Speed", "%.2f", MecanumDrive.DRIVE_SPEED);
        telemetry.addData("Correction", "%.2f", MecanumDrive.CORRECTION_SPEED);
        telemetry.addData("Tolerance", "%.2f", MecanumDrive.TOLERANCE);
        
        telemetry.addData("Pose X (in)", drive.pose.position.x);
        telemetry.addData("Pose Y (in)", drive.pose.position.y);
        telemetry.addData("Heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        
        telemetry.update();
    }
}

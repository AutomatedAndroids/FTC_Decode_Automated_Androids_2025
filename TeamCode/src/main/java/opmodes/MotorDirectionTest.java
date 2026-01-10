package opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import drive.MecanumDrive;

@TeleOp(name = "Motor Direction Test", group = "Test")
public class MotorDirectionTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        telemetry.addLine("Motor Direction Test");
        telemetry.addLine("Hold gamepad buttons to test each motor:");
        telemetry.addLine("A - Front Left (Should spin FORWARD/OUT)");
        telemetry.addLine("B - Front Right (Should spin FORWARD/OUT)");
        telemetry.addLine("X - Back Left (Should spin FORWARD/OUT)");
        telemetry.addLine("Y - Back Right (Should spin FORWARD/OUT)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                drive.leftFront.setPower(0.5);
                telemetry.addData("Running", "Front Left");
            } else {
                drive.leftFront.setPower(0);
            }

            if (gamepad1.b) {
                drive.rightFront.setPower(0.5);
                telemetry.addData("Running", "Front Right");
            } else {
                drive.rightFront.setPower(0);
            }

            if (gamepad1.x) {
                drive.leftBack.setPower(0.5);
                telemetry.addData("Running", "Back Left");
            } else {
                drive.leftBack.setPower(0);
            }

            if (gamepad1.y) {
                drive.rightBack.setPower(0.5);
                telemetry.addData("Running", "Back Right");
            } else {
                drive.rightBack.setPower(0);
            }
            
            drive.updatePoseEstimate();
            telemetry.addData("Pose X", drive.pose.position.x);
            telemetry.addData("Pose Y", drive.pose.position.y);
            telemetry.addData("Heading", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }
    }
}


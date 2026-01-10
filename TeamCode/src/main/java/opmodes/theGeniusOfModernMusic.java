/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package opmodes;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import subsystems.IntakeSubsystem;
import subsystems.MecanumSubsystem;
import subsystems.ShooterSubsystem;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="close blue threat level midnight", group="Robot")
public class theGeniusOfModernMusic extends LinearOpMode {

    /* Declare OpMode members. */
    private MecanumSubsystem drive;

    private DcMotorEx leftFront, leftBack, rightBack, rightFront;

    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFront = hardwareMap.get(DcMotorEx.class, "fL");
        leftBack = hardwareMap.get(DcMotorEx.class, "bL");
        rightBack = hardwareMap.get(DcMotorEx.class, "bR");
        rightFront = hardwareMap.get(DcMotorEx.class, "fR");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: REVERSE MOTORS HERE IF NEEDED
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        try {
            com.arcrobotics.ftclib.hardware.motors.CRServo leftFeeder = new com.arcrobotics.ftclib.hardware.motors.CRServo(hardwareMap, "leftFeeder");
            com.arcrobotics.ftclib.hardware.motors.CRServo rightFeeder = new CRServo(hardwareMap, "rightFeeder");
            MotorEx shooterMotor = new MotorEx(hardwareMap, "shooter");
            shooter = new ShooterSubsystem(leftFeeder, rightFeeder, shooterMotor, telemetry);
        } catch (Exception e) {
            telemetry.addData("Warning", "Shooter failed to init");
        }

        try {
            Motor intakeMotor = new Motor(hardwareMap, "intake");
            Servo sortArm = hardwareMap.get(Servo.class, "sortArm");
            intake = new IntakeSubsystem(intakeMotor, sortArm, telemetry);

        } catch (Exception e) {
            telemetry.addData("Warning", "Intake failed to init");
            telemetry.addData("Warning", e);
        }


        // Wait for the game to start (driver presses START)
        waitForStart();

        shooter.shoot_close();

        sleep(15000);

        rightBack.setPower(-0.45);
        rightFront.setPower(-0.45);
        leftBack.setPower(-0.45);
        leftFront.setPower(-0.45);

        sleep(1500);

        rightBack.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        leftFront.setPower(0);


        shooter.feedLeft();

        sleep(2000);

        shooter.feedRight();

        sleep(2000);

        intake.turnOnIntake();
        shooter.feed();

        sleep(6000);

        shooter.stopFeeding();
        shooter.stopFlywheels();


        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

    }
}
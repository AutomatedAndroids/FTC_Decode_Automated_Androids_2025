package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem extends SubsystemBase{

    private CRServo leftFeeder, rightFeeder;
    private Servo leftSaftey, rightSaftey;
    private MotorEx shooterMotor;
    private Telemetry telemetry;
    private double shootClose = 0.5;
    private double shootFar = 0.6;
    // --- CONFIGURATION ---
    // Change this based on your exact motor! (28 is standard for 1:1 Rev/GoBilda)
    private static final double TICKS_PER_REV = 28.0;

    // The RPM you want to reach
    private static final double TARGET_RPM_CLOSE = 2750;
    private static final double TARGET_RPM_FAR = 3000;
    private static final double SERVO_BOTTOM = 0.25;
    private static final double SERVO_TOP = 0.5;




    public ShooterSubsystem(CRServo leftFeeder, CRServo rightFeeder, MotorEx shooterMotor, Servo leftSaftey, Servo rightSaftey, Telemetry telemetry) {
        this.leftFeeder = leftFeeder;
        this.rightFeeder = rightFeeder;
        this.shooterMotor = shooterMotor;
        this.leftSaftey = leftSaftey;
        this.rightSaftey = rightSaftey;
        this.telemetry = telemetry;

        leftFeeder.setInverted(true);

        shooterMotor.setRunMode(Motor.RunMode.VelocityControl);


        // Configure Motor for Velocity Control

        // Zero Power Behavior (Float is usually better for high-speed flywheels)
//        this.shooter.setRunMode(Motor.RunMode.VelocityControl);
        this.shooterMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        // Note: If the motor vibrates or doesn't reach speed, you may need to tune PIDF.
        this.shooterMotor.setVeloCoefficients(0.05, 0, 0);

        rightFeeder.setInverted(true);

    }

    public void shoot_far() {
        // Re-calculating constants inside the method is often redundant if they are
        // already class constants, but we'll use your local calculation logic for now.

        // Note: Use your class constants (TARGET_VELOCITY) if possible to avoid recalculation.


        // 1. Set the desired speed. The motor's internal PID controller
        // will now try to hit this velocity.
        // --- FIX IS HERE ---
        shooterMotor.setVelocity((TARGET_RPM_FAR / 60.0) * TICKS_PER_REV);

        // 2. Set the power. This acts as the maximum power the PID controller
        // is allowed to use to reach and maintain the target velocity.
        // Set it to 1.0 to give the controller full authority.
//        shooterMotor.set(shootFar);
    }

    public void shoot_close() {
        shooterMotor.setVelocity((TARGET_RPM_CLOSE / 60.0) * TICKS_PER_REV);
    }

    public void feedLeft() {
        leftFeeder.set(1);
        leftSaftey.setPosition(SERVO_BOTTOM);

    }

    public void feedRight() {
        rightFeeder.set(1);
        rightSaftey.setPosition(SERVO_BOTTOM);
    }

    public void feed() {

        leftFeeder.set(1);
        rightFeeder.set(1);
        leftSaftey.setPosition(SERVO_BOTTOM);
        rightSaftey.setPosition(SERVO_BOTTOM);
    }

    public void stopLeft() {
        leftFeeder.set(-0.001);
        // Ensures velocity is cleared
        leftSaftey.setPosition(SERVO_TOP);
    }

    public void stopRight() {
        rightFeeder.set(-0.001);
        // Ensures velocity is cleared
        rightSaftey.setPosition(SERVO_TOP);
    }

    public void stopFeeding() {
        leftFeeder.set(-0.001);
        rightFeeder.set(-0.001);
        // Ensures velocity is cleared
        leftSaftey.setPosition(SERVO_TOP);
        rightSaftey.setPosition(SERVO_TOP);
    }

    public void stopFlywheels() {
        shooterMotor.setVelocity(0);
        // Ensures velocity is cleared
    }

    public void increaseShootClose(){
        shootClose += 0.01;
    }

    public void decreaseShootClose(){
        shootClose -= 0.01;
    }

    public void increaseShootFar(){
        shootFar += 0.01;
    }

    public void decreaseShootFar(){
        shootFar -= 0.01;
    }


    public double getShootClose(){
        return shootClose;
    }

    public double getShootFar() {
        return shootFar;
    }

    /**
     * Gets the current RPM of the shooter motor based on encoder velocity
     * @return RPM of the shooter motor, or 0 if motor is not available
     */
    public double getShooterRPM() {
        if (shooterMotor == null || shooterMotor.motorEx == null) {
            return 0.0;
        }
        // Access the underlying DcMotorEx to get velocity in encoder ticks per second
        DcMotorEx motor = shooterMotor.motorEx;
        double velocityTicksPerSecond = motor.getVelocity();
        // Convert to RPM: (ticks/sec) / (ticks/rev) * (60 sec/min) = RPM
        return (velocityTicksPerSecond / TICKS_PER_REV) * 60.0;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (telemetry != null) {
            telemetry.addData("Shooter Far", shootFar);
            telemetry.addData("Shooter Close", shootClose);
            telemetry.addData("Shooter RPM", String.format("%.2f", getShooterRPM()));
        }
    }
}

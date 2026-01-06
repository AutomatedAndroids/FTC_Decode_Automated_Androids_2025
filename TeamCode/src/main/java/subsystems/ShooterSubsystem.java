package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem extends SubsystemBase{

    private CRServo leftFeeder, rightFeeder;
    private MotorEx shooterMotor;
    private Telemetry telemetry;

    // --- CONFIGURATION ---
    // Change this based on your exact motor! (28 is standard for 1:1 Rev/GoBilda)
    private static final double TICKS_PER_REV = 28.0;

    // The RPM you want to reach
    private static final double TARGET_RPM = 1;

    // Calculate Ticks Per Second: (RPM / 60) * TicksPerRev
    private static final double TARGET_VELOCITY = (TARGET_RPM / 60.0) * TICKS_PER_REV;

    public ShooterSubsystem(CRServo leftFeeder, CRServo rightFeeder, MotorEx shooterMotor, Telemetry telemetry) {
        this.leftFeeder = leftFeeder;
        this.rightFeeder = rightFeeder;
        this.shooterMotor = shooterMotor;
        this.telemetry = telemetry;


        shooterMotor.setRunMode(Motor.RunMode.RawPower);


//        // Configure Motor for Velocity Control
//
//        // Zero Power Behavior (Float is usually better for high-speed flywheels)
//        this.shooter.setRunMode(Motor.RunMode.VelocityControl);
//        this.shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
//        // Note: If the motor vibrates or doesn't reach speed, you may need to tune PIDF.
//        this.shooter.setVeloCoefficients(0.05, 0, 0); Example tuning if needed
    }

    public void shoot_far() {
        // Re-calculating constants inside the method is often redundant if they are
        // already class constants, but we'll use your local calculation logic for now.

        // Note: Use your class constants (TARGET_VELOCITY) if possible to avoid recalculation.


        // 1. Set the desired speed. The motor's internal PID controller
        // will now try to hit this velocity.
        // --- FIX IS HERE ---
//        shooter.setVelocity(TARGET_VELOCITY);

        // 2. Set the power. This acts as the maximum power the PID controller
        // is allowed to use to reach and maintain the target velocity.
        // Set it to 1.0 to give the controller full authority.
        shooterMotor.set(0.65);
    }

    public void shoot_close() {
        shooterMotor.set(0.55);
    }

    public void feedLeft() {
        leftFeeder.set(-1);
    }

    public void feedRight() {
        rightFeeder.set(1);
    }

    public void feed() {

        leftFeeder.set(1);
        rightFeeder.set(1);
    }

    public void stopLeft() {
        leftFeeder.set(0);
        // Ensures velocity is cleared
    }

    public void stopRight() {
        rightFeeder.set(0);
        // Ensures velocity is cleared
    }

    public void stopFeeding() {
        leftFeeder.set(0);
        rightFeeder.set(0);
        // Ensures velocity is cleared
    }

    public void stopFlywheels() {
        shooterMotor.set(0);
        // Ensures velocity is cleared
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}

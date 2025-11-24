package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem extends SubsystemBase{

    private CRServo feeder;
    private MotorEx shooter;
    private Telemetry telemetry;

    // --- CONFIGURATION ---
    // Change this based on your exact motor! (28 is standard for 1:1 Rev/GoBilda)
    private static final double TICKS_PER_REV = 28.0;

    // The RPM you want to reach
    private static final double TARGET_RPM = 1;

    // Calculate Ticks Per Second: (RPM / 60) * TicksPerRev
    private static final double TARGET_VELOCITY = (TARGET_RPM / 60.0) * TICKS_PER_REV;

    public ShooterSubsystem(CRServo feeder, MotorEx shooter, Telemetry telemetry) {
        this.feeder = feeder;
        this.shooter = shooter;
        this.telemetry = telemetry;


        shooter.setRunMode(Motor.RunMode.RawPower);


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
        shooter.set(0.85);
    }

    public void shoot_close() {
        shooter.set(0.65);
    }


    public void feed() {
        feeder.set(1);
    }

    public void stopFeeding() {
        feeder.set(0);
        // Ensures velocity is cleared
    }

    public void stopFlywheels() {
        shooter.set(0);
        // Ensures velocity is cleared
    }

    @Override
    public void periodic() {
        super.periodic();
        telemetry.addData("Is this even running", "yes");
    }
}

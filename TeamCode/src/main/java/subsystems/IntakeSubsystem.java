package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class IntakeSubsystem extends SubsystemBase {
    private Motor intakeMotor;
    private Servo sortArmLeft;
    private Servo sortArmRight;
    private Telemetry telemetry;

    public IntakeSubsystem(Motor intakeMotor, Servo sortArmLeft, Servo sortArmRight, Telemetry telemetry) {
        this.intakeMotor = intakeMotor;
        this.sortArmLeft = sortArmLeft;
        this.sortArmRight = sortArmRight;
        this.telemetry = telemetry;
    }

    public void sort(boolean side) { //false is left, true is right
        if (side) {
            sortArmLeft.setPosition(0.75);
            sortArmRight.setPosition(-0.75);
            telemetry.addData("Sorting to the right", "");
        } else {
            sortArmLeft.setPosition(-0.75);
            sortArmRight.setPosition(0.75);
            telemetry.addData("Sorting to the left", "");
        }
    }

    public void sort(){
        sortArmLeft.setPosition(-0.75);
        sortArmRight.setPosition(-0.75);
    }

    public void turnOnIntake() {
        intakeMotor.set(1);
        telemetry.addData("Intake", "On");
    }

    public void turnOffIntake() {
        intakeMotor.set(0);
        telemetry.addData("Intake", "Off");
    }

    @Override
    public void periodic() {
        telemetry.update();
    }
}

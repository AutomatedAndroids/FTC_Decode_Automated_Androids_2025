package opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import commands.Auto3PushCommand;
import subsystems.IntakeSubsystem;
import subsystems.MecanumDriveSubsystem;
import subsystems.VisionSubsystem;
import util.DashServer;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import com.acmerobotics.roadrunner.Pose2d;

@Autonomous(name = "Simple Auto Push", group = "Autonomous")
public class AutoOpmodeMecanumPathPlan extends CommandOpMode {

    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private VisionSubsystem visionSubsystem;
    private IntakeSubsystem intakeSubsystem;

    ////// DO NOT MODIFY THIS FUNCTION UNLESS YOU GET CONFIRMED!!!
    private int FrameCounter = 0;
    static final double MIN_TASK_RUN_PERIOD = 100;

    @Override
    public void initialize() {
        // Initialize Vision Subsystem (handles AprilTag and Limelight)
        visionSubsystem = new VisionSubsystem(hardwareMap, telemetry);

        // Initialize Drive Subsystem
        mecanumDriveSubsystem = MecanumDriveSubsystem.create(
                hardwareMap,
                new Pose2d(0, 0, 0),  // Start at origin: x=0, y=0, heading=0 radians
                telemetry
        );

        // Connect Limelight to drive subsystem if vision subsystem initialized successfully
        if (visionSubsystem != null && visionSubsystem.getLimelight() != null) {
            mecanumDriveSubsystem.setVisionProcessors(
                    null, // No webcam - using Limelight
                    visionSubsystem.getLimelight()
            );
        }

        mecanumDriveSubsystem.enableDrive();

        // Initialize Intake Subsystem
        try {
            Motor intakeMotor = new Motor(hardwareMap, "intake");
            Servo sortArm = hardwareMap.get(Servo.class, "sortArm");
            intakeSubsystem = new IntakeSubsystem(intakeMotor, sortArm, telemetry);
        } catch (Exception e) {
            telemetry.addData("Warning", "Intake not found/configured");
            telemetry.update();
        }

        // Schedule the autonomous command
        schedule(new Auto3PushCommand(
                mecanumDriveSubsystem,
                intakeSubsystem,
                mecanumDriveSubsystem.getAchievableMaxDistancePerSecond()
        ));
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        DashServer.Init();
        boolean connected = false;
        do {
            connected = DashServer.Connect();
            connected |= DashServer.AddData("time", FrameCounter);
            sleep(1);
        } while (!connected);

        initialize();
        waitForStart();
        double taskRunTime = 0;

        while (!isStopRequested() && opModeIsActive()) {
            DashServer.AddData("tskTime", taskRunTime);
            double currentTime = (double) System.nanoTime() / 1E9;
            DashServer.AddData("OSTime", currentTime);
            run();
            DashServer.AddData("time", FrameCounter++);
            DashServer.AddData("busVoltage",
                    controlHub.getInputVoltage(VoltageUnit.VOLTS));
            DashServer.DashData();

            taskRunTime = (double) System.nanoTime() / 1E9 - currentTime;
            long sleepTime = (long) (MIN_TASK_RUN_PERIOD - taskRunTime * 1000);
            if (sleepTime > 0)
                sleep(sleepTime);
        }
        reset();
        if (visionSubsystem != null) {
            visionSubsystem.stop();
        }
        DashServer.AddData("time", FrameCounter++);
        DashServer.AddData("OSTime", (double) System.nanoTime() / 1E9);
        DashServer.DashData();
        DashServer.Close();
    }
}

package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import subsystems.IntakeSubsystem;
import subsystems.MecanumDriveSubsystem;

/**
 * Simple autonomous command that:
 * 1. Drives forward for 2 seconds
 * 2. Turns on intake
 * 3. Drives forward for 1 more second while intaking
 * 4. Turns off intake
 * 5. Stops
 */
public class Auto3PushCommand extends SequentialCommandGroup {
    
    public Auto3PushCommand(
            MecanumDriveSubsystem driveSubsystem,
            IntakeSubsystem intakeSubsystem,
            double maxDistancePerSecond) {
        
        // Create a sequence of commands
        addCommands(
                // 1. Drive forward for 2 seconds
                new SimpleDriveCommand(driveSubsystem, 0.5, 0, 0, 2.0),
                
                // 2. Turn on intake
                new TurnOnIntakeCommand(intakeSubsystem),
                
                // 3. Drive forward for 1 more second while intaking
                new SimpleDriveCommand(driveSubsystem, 0.5, 0, 0, 1.0),
                
                // 4. Turn off intake
                new TurnOffIntakeCommand(intakeSubsystem),
                
                // 5. Stop (handled by SimpleDriveCommand end)
                new WaitCommand(100) // Small wait to ensure everything stops (100ms)
        );
    }
}

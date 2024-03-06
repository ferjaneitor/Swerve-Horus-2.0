package frc.robot.Macros;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.LimeLight.LimeLightSubsystem;
import frc.robot.commands.AimSpeakerCmd;
import frc.robot.commands.LauncherActivateCmd;

public class SpeakerScoreMac extends SequentialCommandGroup {
    
    public SpeakerScoreMac(SwerveSubsystem swerveSubsystem, LauncherSubsystem launcherSubsystem, LimeLightSubsystem limeLightSubsystem, double speed) {

        addCommands(
            new AimSpeakerCmd(swerveSubsystem, limeLightSubsystem),

            new InstantCommand(() -> limeLightSubsystem.setLimeLed(2)),

            new LauncherActivateCmd(launcherSubsystem, swerveSubsystem, speed),

            new InstantCommand(() -> limeLightSubsystem.setLimeLed(0))
        );
    }
}

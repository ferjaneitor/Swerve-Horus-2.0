package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.commands.LauncherActivateCmd;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutonomousOnlyShooter extends SequentialCommandGroup {

    int kIA = AutoConstants.kColorTeam;

    public AutonomousOnlyShooter(SwerveSubsystem swerveSubsystem, LauncherSubsystem launcherSubsystem) {


        // 5. Add commands to the SequentialCommandGroup
        addCommands(

            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
            
            // Launcher start
            new LauncherActivateCmd(launcherSubsystem, swerveSubsystem, LauncherConstants.kUpperRoller_Speed2),

            // Stop the swerve modules after trajectory is complete
            new InstantCommand(() -> swerveSubsystem.stopModules())
        );
    }
}
package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.commands.LauncherActivateCmd;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import java.util.List;

public class AutonomousFeederSide_Blue extends SequentialCommandGroup {

    int kIA = AutoConstants.kColorTeam;

    public AutonomousFeederSide_Blue(SwerveSubsystem swerveSubsystem, LauncherSubsystem launcherSubsystem) {
        // Asume que necesitas el subsistema de swerve para ejecutar la trayectoria

        // 1. Create trajectory configuration
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(1.0606601717798, 1.0606601717798*kIA),
                new Translation2d(2.1213203436,0)),
            new Pose2d(3.1819805153, -1.0606601717798, Rotation2d.fromDegrees(0)),
            trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            swerveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            swerveSubsystem::setModulesState,
            swerveSubsystem);

        // 5. Add commands to the SequentialCommandGroup
        addCommands(

            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
            
            // Launcher start
            new LauncherActivateCmd(launcherSubsystem, swerveSubsystem, LauncherConstants.kUpperRoller_Speed2),
            
            // Follow the trajectory
            swerveControllerCommand,

            // Reset odometry to trajectory's initial pose
            new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),

            // Stop the swerve modules after trajectory is complete
            new InstantCommand(() -> swerveSubsystem.stopModules())
        );
    }
}
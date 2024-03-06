package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LimeLight.LimeLightSubsystem;
import frc.robot.Autonomous.AutonomousCenter;
import frc.robot.Autonomous.AutonomousFeederSide_Blue;
import frc.robot.Autonomous.AutonomousFeederSide_Red;
import frc.robot.Autonomous.AutonomousOnlyShooter;
import frc.robot.Autonomous.AutoTest;

import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.LauncherActivateCmd;
import frc.robot.commands.LauncherFeederCmd;
import frc.robot.commands.LauncherLoadingCmd;
import frc.robot.commands.AimSpeakerCmd;
import frc.robot.commands.AimAmpCmd;
import frc.robot.commands.ClawCmd;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.Macros.SpeakerScoreMac;


public class RobotContainer {
  
  //private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
  private final ClawSubsystem clawSubsystem= new ClawSubsystem();
  private final LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem();

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick shooterController = new Joystick(OIConstants.kShooterControllerPort);
  
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

      m_autoChooser.addOption("DESHABILITADO", null);
      m_autoChooser.addOption("Autónomo CENTRAL", new AutonomousCenter(swerveSubsystem, launcherSubsystem));
      m_autoChooser.addOption("Autónomo Feeder BLUE", new AutonomousFeederSide_Blue(swerveSubsystem, launcherSubsystem));
      m_autoChooser.addOption("Autónomo Feeder RED", new AutonomousFeederSide_Red(swerveSubsystem, launcherSubsystem));
      m_autoChooser.addOption("ONLY SHOOTER", new AutonomousOnlyShooter(swerveSubsystem, launcherSubsystem));
    
    // Publica el selector en el SmartDashboard
    SmartDashboard.putData("Selector Autónomo", m_autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverJoystick, 2) // Resetea el frente del chasis (B)
            .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

    new JoystickButton(driverJoystick, 3) // Auto alinearse al Speaker y Disparar 75 (X)
            .toggleOnTrue(new SpeakerScoreMac(swerveSubsystem, launcherSubsystem, limeLightSubsystem, LauncherConstants.kUpperRoller_Speed1));
    new JoystickButton(driverJoystick, 4) // Auto alinearse al Speaker y Disparar 85 (X)
            .toggleOnTrue(new SpeakerScoreMac(swerveSubsystem, launcherSubsystem, limeLightSubsystem, LauncherConstants.kUpperRoller_Speed2));


    new JoystickButton(shooterController, 1) // Launcher con velocidad 75 (A)
            .onTrue(new LauncherActivateCmd(launcherSubsystem, swerveSubsystem, LauncherConstants.kUpperRoller_Speed1));
    new JoystickButton(shooterController, 4) // Launcher con velocidad 85 (Y)
            .onTrue(new LauncherActivateCmd(launcherSubsystem, swerveSubsystem, LauncherConstants.kUpperRoller_Speed2));
    new JoystickButton(shooterController, 2) // Activa el feeder (B)
            .whileTrue(new LauncherFeederCmd(launcherSubsystem, LauncherConstants.kFeederRollers_Speed, limeLightSubsystem));
    new JoystickButton(shooterController, 3) // Activa el loader (X) 
            .onTrue(new LauncherLoadingCmd(launcherSubsystem, LauncherConstants.kLoadingRollers_Speed, limeLightSubsystem));
    
    new Trigger(() -> shooterController.getRawAxis(ClawConstants.kClawInFast_Button) > 0.5) // In Fast (LT)
            .whileTrue(new ClawCmd(clawSubsystem, ClawConstants.kClawInFast_Speed, limeLightSubsystem));
    new Trigger(() -> shooterController.getRawAxis(ClawConstants.kClawOutFast_Button) > 0.5) // Out Fast (RT)
            .whileTrue(new ClawCmd(clawSubsystem, ClawConstants.kClawOutFast_Speed, limeLightSubsystem));
    new JoystickButton(shooterController, ClawConstants.kClawInSlow_Button) // In slow (LB)
            .whileTrue(new ClawCmd(clawSubsystem, ClawConstants.kClawInSlow_Speed, limeLightSubsystem));
    new JoystickButton(shooterController, ClawConstants.kClawOutSlow_Button) // Out slow (RB)
            .whileTrue(new ClawCmd(clawSubsystem, ClawConstants.kClawOutSlow_Speed, limeLightSubsystem));
    
    
  }

  public void m_resetEncoders() {
        swerveSubsystem.s_resetEncoders();
  }

  public Command getAutonomousCommand() {

    return m_autoChooser.getSelected();
      
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import static edu.wpi.first.units.Units.RPM;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  String deployDirectory = (Robot.isSimulation()) ? "sim-swerve/neo" : "swerve";
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), deployDirectory));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  SendableChooser<String> m_chooser = new SendableChooser<>();      
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    SmartDashboard.putData("autoChooser", m_chooser);

    m_chooser.addOption("Taxi Port", "Taxi Port");
    m_chooser.addOption("Taxi Center", "Taxi Center");
    
    // Configure the trigger bindings
    configureBindings();

    // Set the default command to force the shooter rest.
    m_shooterSubsystem.setDefaultCommand(m_shooterSubsystem.set(0));

    Command driveFieldOrientedAngularVelocity = getTeleopDriveCommand();
    swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

  private Command getTeleopDriveCommand() {
    Command driveFieldOrientedAngularVelocity = swerveSubsystem.driveCommand(
        () -> {
          double y = MathUtil.applyDeadband(
            m_driverController.getLeftY(),
              OperatorConstants.LEFT_Y_DEADBAND);
          if (!PoseTransformUtils.isRedAlliance()) {
            y *= -1;
          }
          return swerveSubsystem.m_yspeedLimiter.calculate(y);
        },
        () -> {
          double x = MathUtil.applyDeadband(
            m_driverController.getLeftX(),
              OperatorConstants.LEFT_X_DEADBAND);
          if (!PoseTransformUtils.isRedAlliance()) {
            x *= -1;
          }
          return swerveSubsystem.m_xspeedLimiter.calculate(x);
        },
        () -> -m_driverController.getRightX());
    return driveFieldOrientedAngularVelocity; // :P
  }

  private void configureBindings() {

    // Schedule `setVelocity` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.a().whileTrue(m_shooterSubsystem.setRotationVelocity(RPM.of(60)));
    m_driverController.b().whileTrue(m_shooterSubsystem.setRotationVelocity(RPM.of(-60)));
    // Schedule `set` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.x().whileTrue(m_shooterSubsystem.set(0.3));
    // m_driverController.y().whileTrue(m_shooterSubsystem.set(-0.3));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
      * @param string 
      *
      * @return the command to run in autonomous
      */
     public Command getAutonomousCommand(String auto) {
      // An example command will be run in autonomous
      return swerveSubsystem.getAutonomousCommand(auto);
     }

    public Command getAutonomousCommand() {
      // An example command will be run in autonomous
      return getAutonomousCommand(m_chooser.getSelected());
    }
}

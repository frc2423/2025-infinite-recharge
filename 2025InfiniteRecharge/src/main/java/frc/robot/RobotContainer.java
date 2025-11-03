// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ArmSubsystem m_exampleSubsystem = new ArmSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Set the default command to force the arm to go to 0.
    m_exampleSubsystem.setDefaultCommand(m_exampleSubsystem.setAngle(Degrees.of(0)));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    m_exampleSubsystem.getArm().getAngle().lt(Degrees.of(0));
    
    var down = Commands.sequence(m_exampleSubsystem.setAngle(Degrees.of(15)).until(() -> m_exampleSubsystem.getArm().getAngle().gt(Degrees.of(14.9))), m_exampleSubsystem.setAngle(Degrees.of(-22)).withTimeout(1.5));
    // Schedule `setAngle` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.a().whileTrue(down);
    m_driverController.b().whileTrue(m_exampleSubsystem.setAngle(Degrees.of(15)));
    // Schedule `set` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.x().whileTrue(m_exampleSubsystem.set(0.3));
    m_driverController.y().whileTrue(m_exampleSubsystem.set(-0.3));


    Commands.run(() -> {
      System.out.println("Hello World");
    }).until(() -> {
      return true;
    });

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(ArmSubsystem);
    return null;
  }
}

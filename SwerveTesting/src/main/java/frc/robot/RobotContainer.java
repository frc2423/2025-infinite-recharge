// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;

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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController driveController = new XboxController(
      OperatorConstants.kDriverControllerPort);

  private final CANBus canBus = new CANBus("Jerry");

  private final Pigeon2 pidgey = new Pigeon2(13, canBus);

  private final TalonFX talonfx1 = new TalonFX(1, canBus);
  private final TalonFX talonfx2 = new TalonFX(2, canBus);
  private final TalonFX talonfx3 = new TalonFX(3, canBus);
  private final TalonFX talonfx4 = new TalonFX(4, canBus);
  private final TalonFX talonfx5 = new TalonFX(5, canBus);
  private final TalonFX talonfx6 = new TalonFX(6, canBus);
  private final TalonFX talonfx7 = new TalonFX(7, canBus);
  private final TalonFX talonfx8 = new TalonFX(8, canBus);

  private final CANcoder C1 = new CANcoder(9, canBus);
  private final CANcoder C2 = new CANcoder(10, canBus);
  private final CANcoder C3 = new CANcoder(11, canBus);
  private final CANcoder C4 = new CANcoder(12, canBus);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public void testSwerve() {

    NTHelper.setDouble("/swerveSubsystem/pigeon/yaw", pidgey.getYaw().getValueAsDouble());

    NTHelper.setDouble("/swerveSubsystem/CANCoder/C1",
        C1.getPosition().getValueAsDouble());
    NTHelper.setDouble("/swerveSubsystem/CANCoder/C2",
        C2.getPosition().getValueAsDouble());
    NTHelper.setDouble("/swerveSubsystem/CANCoder/C3",
        C3.getPosition().getValueAsDouble());
    NTHelper.setDouble("/swerveSubsystem/CANCoder/C4",
        C4.getPosition().getValueAsDouble());

    double speed = driveController.getLeftY();
    double turn = driveController.getRightX();

    talonfx1.set(speed);
    talonfx3.set(speed);
    talonfx5.set(speed);
    talonfx7.set(speed);
    talonfx2.set(turn);
    talonfx4.set(turn);
    talonfx6.set(turn);
    talonfx8.set(turn);

  }

}

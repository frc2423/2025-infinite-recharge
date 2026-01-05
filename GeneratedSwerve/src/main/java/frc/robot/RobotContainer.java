// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.PrintStream;
import java.lang.annotation.Target;

import javax.xml.xpath.XPathFactory;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.rightBumper()
                .whileTrue(goToPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(-0, 0)))); // Pass
                                                                                                  // in a
                                                                                                  // pose2d
                                                                                                  // to
                                                                                                  // move
                                                                                                  // to
                                                                                                  // location
                                                                                                  // on
                                                                                                  // rightbumper
                                                                                                  // hold

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public Command goToPose(Pose2d target) {

        Command moveAndSlide = drivetrain.applyRequest(() -> { // godot reference 🤣, 😜, 🤔, and 🤡

            var whereami = drivetrain.getState().Pose; // Get pose

            double starting_x = whereami.getX();
            double starting_y = whereami.getY();
            Rotation2d starting_r = whereami.getRotation();
            double target_x = target.getX();
            double target_y = target.getY();
            Rotation2d target_r = target.getRotation();

            double x = target_x - starting_x;
            double y = target_y - starting_y;
            double r = target_r.minus(starting_r).getRadians();

            double d = Math.sqrt((x * x) + (y * y));

            double x_norm;
            double y_norm;

            if (d > 0.001) {
                x_norm = x / d;
                y_norm = y / d;
            } else {
                x_norm = 0.00;
                y_norm = 0.00;
            }
            ;
            // System.out.println(r);

            double rotation = 0;

            if (r < 0.03 && r > -0.03) {
                rotation = 0;
            } else {
                rotation = r * 3 + Math.copySign(.475, r);
            }

            // Please stop

            double speed_coefficent = Math.min(.3 + 2 * d, 1);
            // System.out.println(
            // "X norm: " + x_norm + "Y norm: " + y_norm + ", speed: " + speed_coefficent +
            // ", distance: " + d);
            return drive.withVelocityX(speed_coefficent * -x_norm)
                    .withVelocityY(speed_coefficent * -y_norm).withDeadband(0)
                    .withRotationalRate(rotation).withRotationalDeadband(0);
        });

        return moveAndSlide;
    }

}

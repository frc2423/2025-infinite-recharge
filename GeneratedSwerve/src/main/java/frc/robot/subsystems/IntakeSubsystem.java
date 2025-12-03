package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

public class IntakeSubsystem extends SubsystemBase {
    
    private SparkFlex intake = new SparkFlex(18, MotorType.kBrushless);

    public IntakeSubsystem() {
        setDefaultCommand(intakeStop());
    }

    public Command intakeIn() {
        var command = run(()->intake.set(0.5));
        command.setName("Intake In");

        return command;
    }

    public Command intakeOut() {
        var command = run(()->intake.set(-0.5));
        command.setName("Intake Out");

        return command;
    }

    public Command intakeStop() {
        var command = runOnce(()->intake.set(0));
        command.setName("Intake Stop");

        return command;
    }
}

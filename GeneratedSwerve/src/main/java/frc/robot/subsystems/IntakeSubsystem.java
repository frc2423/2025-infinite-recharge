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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;

public class IntakeSubsystem extends SubsystemBase {

    private SparkFlex intake = new SparkFlex(18, MotorType.kBrushless);
    SparkFlexConfig motorConfig = new SparkFlexConfig();

    public IntakeSubsystem() {
        setCurrentLimit(60, 40);
        setDefaultCommand(intakeStop());
    }

    private void setCurrentLimit(int stallLimit, int freeLimit) {
        motorConfig.smartCurrentLimit(stallLimit, freeLimit);
        intake.configureAsync(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command intakeCheckBroken() {
        var command = runOnce(() -> intake.set(0.5));
        command.setName("nothing wrong");

        if (intake.getOutputCurrent() > 55 /* && intake.get() <= 0.2 */) {
            var trueCommand = run(() -> intake.set(-0.5));
            trueCommand.setName("Intake Out");
            return trueCommand;
        }
        return command;
    }

    public Command intakeIn() {
        var command = run(() -> intake.set(0.5)).until(() -> {
            if (intake.getOutputCurrent() > 55 /* && intake.get() <= 0.2 */) {
                return true;
            }
            return false;
        }).andThen(run(() -> intake.set(-0.5)));

        command.setName("Intake In");

        // if (intake.getOutputCurrent() > 0.2 && intake.get() <= 0.2) {
        // var trueCommand = run(() -> intake.set(-0.5));
        // trueCommand.setName("Intake Out");

        // System.out.println("MADE IT AHHH");
        // return trueCommand;
        // }

        return command;
    }

    public Command intakeOut() {
        var command = run(() -> intake.set(-0.5));
        command.setName("Intake Out");

        return command;
    }

    public Command intakeStop() {
        var command = runOnce(() -> intake.set(0));
        command.setName("Intake Stop");

        return command;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // This is used to add things to NetworkTables
        super.initSendable(builder);

        builder.addDoubleProperty("intakeSpeed", () -> intake.get(), null);
        builder.addDoubleProperty("intakeCurrent", () -> intake.getOutputCurrent(), null);
    }
}

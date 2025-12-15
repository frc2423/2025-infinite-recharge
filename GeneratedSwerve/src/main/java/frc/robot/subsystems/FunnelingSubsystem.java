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

public class FunnelingSubsystem extends SubsystemBase {

    private SparkFlex funnel1 = new SparkFlex(20, MotorType.kBrushless);
    private SparkFlex funnel2 = new SparkFlex(21, MotorType.kBrushless);

    public FunnelingSubsystem() {
        setDefaultCommand(funnelStop());
    }

    public Command funnelIn() {
        var command = run(()->funnel1.set(0.5)).alongWith(run(()->funnel2.set(-0.5)));
        command.setName("Funnel In");

        return command;
    }

    public Command funnelStop() {
        var command = runOnce(()->funnel1.set(0)).alongWith(runOnce(()->funnel2.set(0)));
        command.setName("Funnel Stop");

        return command;
    }
}

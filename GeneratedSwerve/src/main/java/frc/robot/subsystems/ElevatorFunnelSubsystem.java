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
import frc.robot.Robot;
import yams.motorcontrollers.simulation.Sensor;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.simulation.MockLaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;

public class ElevatorFunnelSubsystem extends SubsystemBase {

    private SparkFlex elevatorFunnelSubsystem = new SparkFlex(22, MotorType.kBrushless);
    private final LaserCanInterface intakeDist;

    public ElevatorFunnelSubsystem() {
        setDefaultCommand(elevatorFunnelStop());
        intakeDist = Robot.isSimulation() ? new MockLaserCan() : new LaserCan(26);
    }

    public boolean seesBall() {
        return intakeDist.getMeasurement().distance_mm < 250;
    }

    public Command elevatorFunnelIn() {
        var command = run(()->elevatorFunnelSubsystem.set(0.5));
        command.setName("Elevator Funnel In");

        return command;
    }

    

    public Command elevatorFunnelStop() {
        var command = runOnce(()->elevatorFunnelSubsystem.set(0));
        command.setName("Intake Stop");

        return command;
    }

    public Command elevatorFunnelStopIfBroken() {
        var command = runOnce(()->elevatorFunnelSubsystem.set(0));
        command.setName("Intake Stop If Broken");

        return command;
    }
}
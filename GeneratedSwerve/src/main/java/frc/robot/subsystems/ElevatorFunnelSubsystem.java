package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import com.revrobotics.spark.SparkMax;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.simulation.MockLaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

@Logged
public class ElevatorFunnelSubsystem extends SubsystemBase {

    private SparkMax elevatorFunnelSubsystem = new SparkMax(22, MotorType.kBrushless);
    private final LaserCanInterface intakeDist;

    private double speed = 0;

    public ElevatorFunnelSubsystem() {
        setDefaultCommand(elevatorFunnelStop());
        intakeDist = Robot.isSimulation() ? new MockLaserCan() : new LaserCan(26);
    }

    public boolean seesBall() {
        Measurement measurement = intakeDist.getMeasurement();
        if (measurement == null) {
            return false;
        }
        return measurement.distance_mm < 250;
    }

    public void periodic() {
        elevatorFunnelSubsystem.set(speed);
    }

    public Command elevatorFunnelIn() {
        var command = run(()-> {
            speed = -.5;
        });
        command.setName("Elevator Funnel In");

        return command;
    }

    public Command elevatorShooterIn() {
        var command = run(()-> {
            speed = -1;
        });
        command.setName("Elevator Funnel In");

        return command;
    }

    public Command elevatorFunnelStop() {
        var command = runOnce(()-> {
            speed = 0;
        });
        command.setName("Intake Stop");

        return command;
    }

    public Command elevatorFunnelStopIfBroken() {
        var command = runOnce(()-> {
            speed = 0;
        });
        command.setName("Intake Stop If Broken");

        return command;
    }
}
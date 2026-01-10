package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;

@Logged
public class ShooterSubsystem extends SubsystemBase {
    
    private SparkFlex shooter = new SparkFlex(23, MotorType.kBrushless);

    private double speed = 0;

    public ShooterSubsystem() {
        setDefaultCommand(shooterStop());
    }

    public void periodic() {
        shooter.set(speed);
    }

    public Command shooterFast() {
        var command = run(()-> {
            speed = 0.6;
        });
        command.setName("Shooter Fast");

        return command;
    }

    public Command shooterSlow() {
        var command = run(()-> {
            speed = 0.3;
        });
        command.setName("Shooter Slow");

        return command;
    }

    public Command shooterStop() {
        var command = runOnce(()-> {
            speed = 0;
        });
        command.setName("Shooter Stop");

        return command;
    }
}
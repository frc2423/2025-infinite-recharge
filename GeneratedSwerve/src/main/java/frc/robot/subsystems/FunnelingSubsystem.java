package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;

@Logged
public class FunnelingSubsystem extends SubsystemBase {

    private SparkFlex funnelLeft = new SparkFlex(24, MotorType.kBrushless);
    private SparkFlex funnelRight = new SparkFlex(26, MotorType.kBrushless);

    private double speed = 0;

    public FunnelingSubsystem() {
        setDefaultCommand(funnelStop());
    }

    public void periodic() {
        funnelLeft.set(speed);
        funnelRight.set(speed);
    }

    public Command funnelIn() {

        var command = run(() -> {
            speed = -0.5;
        });
        command.setName("Funnel In");

        return command;
    }

    public Command funnelStop() {
        var command = runOnce(() -> {
            speed = 0;
        });
        command.setName("Funnel Stop");

        return command;
    }
}

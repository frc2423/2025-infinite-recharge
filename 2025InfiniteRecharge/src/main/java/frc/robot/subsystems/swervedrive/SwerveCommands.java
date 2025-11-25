package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.controller.PIDController;

public class SwerveCommands {
    
    private SwerveSubsystem swerve;
    PIDController translationPIDX = new PIDController(3.5, 0, .3);
    PIDController translationPIDY = new PIDController(3.5, 0, .3);

    public SwerveCommands(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    public Command GoTO(Pose2d pose2d) {
        var command = Commands.run(() ->{
            double x = translationPIDX.calculate(
                swerve.getPose().getX(), pose2d.getX());

            double xDistance = Math.abs(pose2d.getX() - swerve.getPose().getX());
            
            if (xDistance > 0.0508) {
                x = Math.copySign(Math.max(.35, Math.abs(x)), x);
            }

        }).until(() -> {return pose2d.getX() == 2;});
        
        return command;
    }
}

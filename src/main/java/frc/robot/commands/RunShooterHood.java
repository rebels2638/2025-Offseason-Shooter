package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class RunShooterHood extends Command {
    private final Shooter shooter = Shooter.getInstance();
    private final Rotation2d angle;

    // no requirements because it is expect that a requirement is added in the parent command 
    // this allows for multiple parallel shooter calls to different shooter motors
    public RunShooterHood(Rotation2d angle) {
        this.angle = angle;
    }

    @Override
    public void initialize() {
        shooter.setAngle(angle);
    }

    @Override
    public boolean isFinished() {
        return shooter.isHoodAtSetpoint();
    }
}

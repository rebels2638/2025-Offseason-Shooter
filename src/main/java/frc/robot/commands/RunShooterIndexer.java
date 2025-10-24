package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class RunShooterIndexer extends Command {
    private final Shooter shooter = Shooter.getInstance();
    private final double velocityRotationsPerSec;

    // no requirements because it is expect that a requirement is added in the parent command 
    // this allows for multiple parallel shooter calls to different shooter motors
    public RunShooterIndexer(double velocityRotationsPerSec) {
        this.velocityRotationsPerSec = velocityRotationsPerSec;
    }

    @Override
    public void initialize() {
        shooter.setIndexerVelocity(velocityRotationsPerSec);
    }

    @Override
    public void execute() {
        shooter.setIndexerVelocity(velocityRotationsPerSec);
    }

    @Override
    public boolean isFinished() {
        return shooter.isIndexerAtSetpoint();
    }
}

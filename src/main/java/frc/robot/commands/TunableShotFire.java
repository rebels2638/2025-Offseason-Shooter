package frc.robot.commands;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.VisualizeShot;
import frc.robot.subsystems.shooter.Shooter;

public class TunableShotFire extends SequentialCommandGroup {
    
    private static final Shooter shooter = Shooter.getInstance();
    private static final LoggedNetworkNumber indexerVelocityDashboardNumber = new LoggedNetworkNumber("TunableShot/indexerVelocityRotationsPerSec", 35);

    public TunableShotFire() {
        super(
            new ParallelDeadlineGroup(
                new WaitCommand(0.5),
                new RunShooterIndexer(indexerVelocityDashboardNumber.get())
            ),
            new RunShooterIndexer(0),
            new InstantCommand(() -> new VisualizeShot())
        );
        addRequirements(shooter);
    }
}

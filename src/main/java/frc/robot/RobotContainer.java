package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.shooter.Shooter;


public class RobotContainer {
    public static RobotContainer instance = null;

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private final XboxController xboxTester;
    private final XboxController xboxDriver;
    private final XboxController xboxOperator;

    private final Shooter shooter = Shooter.getInstance();


    private RobotContainer() {
        this.xboxTester = new XboxController(1);
        this.xboxOperator = new XboxController(2);
        this.xboxDriver = new XboxController(3);
    }

    public Command getAutonomousCommand() {
        // return new SequentialCommandGroup(
        //     new InstantCommand(() -> shooter.setAngle(Rotation2d.fromDegrees(0))),
        //     new InstantCommand(() -> shooter.setShotVelocity(2)),
        //     new InstantCommand(() -> shooter.setFeedVelocity(2)),
        //     new WaitCommand(4),
        //     new InstantCommand(() -> shooter.setAngle(Rotation2d.fromDegrees(90))),
        //     new InstantCommand(() -> shooter.setShotVelocity(1)),
        //     new InstantCommand(() -> shooter.setFeedVelocity(1)),
        //     new WaitCommand(4),
        //     new InstantCommand(() -> shooter.setAngle(Rotation2d.fromDegrees(180))),
        //     new InstantCommand(() -> shooter.setShotVelocity(0)),
        //     new InstantCommand(() -> shooter.setFeedVelocity(0)),
        //     new WaitCommand(4),
        //     new InstantCommand(() -> shooter.setAngle(Rotation2d.fromDegrees(270))),
        //     new WaitCommand(4),
        //     new InstantCommand(() -> shooter.setAngle(Rotation2d.fromDegrees(360)))
        // );

        return new SequentialCommandGroup(
            new InstantCommand(() -> shooter.setAngle(Rotation2d.fromDegrees(40))),
            new WaitCommand(1.5),
            new InstantCommand(() -> shooter.setAngle(Rotation2d.fromDegrees(50))),
            new WaitCommand(1.5),
            new InstantCommand(() -> shooter.setAngle(Rotation2d.fromDegrees(60))),
            new WaitCommand(1.5),
            new InstantCommand(() -> shooter.setAngle(Rotation2d.fromDegrees(100))),
            new WaitCommand(1.5),
            new InstantCommand(() -> shooter.setAngle(Rotation2d.fromDegrees(125))),
            new WaitCommand(1.5),
            new InstantCommand(() -> shooter.setAngle(Rotation2d.fromDegrees(170))),
            new WaitCommand(1.5),
            new InstantCommand(() -> shooter.setAngle(Rotation2d.fromDegrees(38)))
        );   
    }
}

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;


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
    // private final RobotState robotState = RobotState.getInstance();
    // private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    // private final Vision vision = Vision.getInstance();

    private RobotContainer() {
        this.xboxTester = new XboxController(1);
        this.xboxOperator = new XboxController(2);
        this.xboxDriver = new XboxController(3);

        // swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(xboxDriver));
        // xboxDriver.getXButton().onTrue(new InstantCommand(() -> robotState.zeroGyro()));
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> shooter.setAngle(Rotation2d.fromDegrees(40))),
            new InstantCommand(() -> shooter.setShotVelocity(1)),
            new InstantCommand(() -> shooter.setFeedVelocity(1)),
            new InstantCommand(() -> shooter.setIndexerVelocity(5)),

            new WaitCommand(1.5),

            new InstantCommand(() -> shooter.setAngle(Rotation2d.fromDegrees(50))),
            new InstantCommand(() -> shooter.setShotVelocity(5)),
            new InstantCommand(() -> shooter.setFeedVelocity(2)),
            new InstantCommand(() -> shooter.setIndexerVelocity(5)),
            new WaitCommand(1.5),

            new InstantCommand(() -> shooter.setAngle(Rotation2d.fromDegrees(60))),
            new InstantCommand(() -> shooter.setShotVelocity(20)),
            new InstantCommand(() -> shooter.setFeedVelocity(10)),
            new InstantCommand(() -> shooter.setIndexerVelocity(10)),
            new WaitCommand(1.5),

            new InstantCommand(() -> shooter.setAngle(Rotation2d.fromDegrees(100))),
            new InstantCommand(() -> shooter.setShotVelocity(40)),
            new InstantCommand(() -> shooter.setFeedVelocity(15)),
            new InstantCommand(() -> shooter.setIndexerVelocity(15)),
            new WaitCommand(1.5),
            
            new InstantCommand(() -> shooter.setAngle(Rotation2d.fromDegrees(125))),
            new InstantCommand(() -> shooter.setShotVelocity(50)),
            new InstantCommand(() -> shooter.setFeedVelocity(25)),
            new InstantCommand(() -> shooter.setIndexerVelocity(25)),
            new WaitCommand(1.5),

            new InstantCommand(() -> shooter.setAngle(Rotation2d.fromDegrees(170))),
            new InstantCommand(() -> shooter.setShotVelocity(60)),
            new InstantCommand(() -> shooter.setFeedVelocity(30)),
            new InstantCommand(() -> shooter.setIndexerVelocity(30)),
            new WaitCommand(1.5),

            new InstantCommand(() -> shooter.setAngle(Rotation2d.fromDegrees(38))),
            new InstantCommand(() -> shooter.setShotVelocity(0)),
            new InstantCommand(() -> shooter.setFeedVelocity(0)),
            new InstantCommand(() -> shooter.setIndexerVelocity(0))
        );   
    }
}

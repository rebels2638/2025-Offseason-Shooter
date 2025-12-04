package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.RunShooterFeeder;
import frc.robot.commands.RunShooterFlywheel;
import frc.robot.commands.RunShooterHood;
import frc.robot.commands.RunShooterIndexer;
import frc.robot.commands.WindupAndShoot;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.FollowPath.Builder;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.Waypoint;
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
    private final RobotState robotState = RobotState.getInstance();
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Vision vision = Vision.getInstance();

    private final AbsoluteFieldDrive absoluteFieldDrive;

    private RobotContainer() {
        this.xboxTester = new XboxController(1);
        this.xboxOperator = new XboxController(2);
        this.xboxDriver = new XboxController(3);
        
        AbsoluteFieldDrive absoluteFieldDrive = new AbsoluteFieldDrive(xboxDriver);
        this.absoluteFieldDrive = absoluteFieldDrive;
        // Command movingShotWindup = new MovingShotWindup(new Translation3d(5, 10, 0), absoluteFieldDrive.getDesiredFieldRelativeSpeedsSupplier(), 5);
        swerveDrive.setDefaultCommand(absoluteFieldDrive);
        // shooter.setDefaultCommand(movingShotWindup);

        this.xboxDriver.getAButton().whileTrue(
            new WindupAndShoot(absoluteFieldDrive.getDesiredFieldRelativeSpeedsSupplier())
        ).onFalse(
            new SequentialCommandGroup(
                new WaitCommand(1),
                new ParallelCommandGroup(
                    new RunShooterIndexer(0),
                    new RunShooterFlywheel(0),
                    new RunShooterFeeder(0),
                    new RunShooterHood(Rotation2d.fromDegrees(45))
                )
            )
        );

        FollowPath.Builder onTheFlyBuilder = new Builder(
            swerveDrive,
            robotState::getEstimatedPose,
            robotState::getRobotRelativeSpeeds,
            swerveDrive::driveRobotRelative,
            new PIDController(6.3, 0, 0),
            new PIDController(12, 0, 1.1),
            new PIDController(3, 0, 0)
        ).withDefaultShouldFlip();



        Path p = new Path(
            new PathConstraints().setMaxVelocityMetersPerSec(3),
            new TranslationTarget(5.4, 5.4),
            new Waypoint(new Translation2d(5, 5), 0.05, new Rotation2d(0))

        );

        this.xboxDriver.getBButton().whileTrue(
            onTheFlyBuilder.build(p)
        );

       

        // this.xboxDriver.getAButton().onTrue(
        //     new TunableShotWindup()
        // );
        // this.xboxDriver.getBButton().onTrue(
        //     new TunableShotFire()
        // );

        // xboxDriver.getXButton().onTrue(new InstantCommand(() -> robotState.zeroGyro()));
    }

    public void teleopInit() {
        // Command command = 
        // command.schedule();
    }

    public Command getAlignModulesCommand(Path path) {
        return new WaitUntilCommand(() -> swerveDrive.alignModules(path.getInitialModuleDirection(), 15).get());
    }
    
    public Command getAutonomousCommand() {
        Path path = new Path("new_path");

        FollowPath.Builder autoBuilder = new Builder(
            swerveDrive,
            robotState::getEstimatedPose,
            robotState::getRobotRelativeSpeeds,
            swerveDrive::driveRobotRelative,
            new PIDController(6.3, 0, 0),
            new PIDController(12, 0, 1.1),
            new PIDController(3, 0, 0)
        ).withDefaultShouldFlip().withPoseReset(robotState::resetPose);

        return getAlignModulesCommand(path).andThen(autoBuilder.build(path));
    }
}

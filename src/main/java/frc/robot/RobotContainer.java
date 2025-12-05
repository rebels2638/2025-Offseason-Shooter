package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.Constants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.DesiredState;
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
    private final Superstructure superstructure = Superstructure.getInstance();
    private final Vision vision = Vision.getInstance();

    // Path for follow path state
    private Path currentPath = null;

    private RobotContainer() {
        this.xboxTester = new XboxController(1);
        this.xboxOperator = new XboxController(2);
        this.xboxDriver = new XboxController(3);
        
        // Configure teleop input suppliers for SwerveDrive FSM
        // Using normalized inputs (-1 to 1) with deadband applied
        swerveDrive.setTeleopInputSuppliers(
            () -> -MathUtil.applyDeadband(xboxDriver.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND),
            () -> -MathUtil.applyDeadband(xboxDriver.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND),
            () -> -MathUtil.applyDeadband(xboxDriver.getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND)
        );

        // Set up path supplier for SwerveDrive
        swerveDrive.setPathSupplier(() -> currentPath);

        // Set default state to TELEOP
        swerveDrive.setDesiredState(SwerveDrive.DesiredState.TELEOP);
        
        // Set default superstructure state to HOME
        superstructure.setDesiredState(Superstructure.DesiredState.HOME);

        configureBindings();
    }

    private void configureBindings() {
        xboxDriver.getAButton().onTrue(
            new InstantCommand(() -> superstructure.setDesiredState(Superstructure.DesiredState.SHOOTING))
        ).onFalse(
            new InstantCommand(() -> superstructure.setDesiredState(Superstructure.DesiredState.READY_FOR_SHOT))
        );

        xboxDriver.getXButton().onTrue(
            new InstantCommand(() -> superstructure.setDesiredState(Superstructure.DesiredState.TRACKING))
        );

        xboxDriver.getYButton().onTrue(
            new InstantCommand(() -> superstructure.setDesiredState(Superstructure.DesiredState.HOME))
        );
    }

    public void teleopInit() {
        // Ensure we're in teleop state
        swerveDrive.setDesiredState(SwerveDrive.DesiredState.TELEOP);
        superstructure.setDesiredState(Superstructure.DesiredState.HOME);
    }

    public void autonomousInit() {
        // Set up for autonomous
        // swerveDrive.setDesiredState(SwerveDrive.DesiredState.PREPARE_FOR_AUTO);
    }

    public void disabledInit() {
        superstructure.setDesiredState(Superstructure.DesiredState.STOPPED);
        swerveDrive.setDesiredState(SwerveDrive.DesiredState.STOPPED);
    }

    public Command getAlignModulesCommand(Path path) {
        return new WaitUntilCommand(() -> swerveDrive.alignModules(path.getInitialModuleDirection(), 15).get());
    }
    
    public Command getAutonomousCommand() {
        // Path path = new Path("new_path");

        // // Use the builder from SwerveDrive with pose reset
        // FollowPath.Builder autoBuilder = swerveDrive.getFollowPathBuilder()
        //     .withPoseReset(robotState::resetPose);

        // return getAlignModulesCommand(path).andThen(
        //     new InstantCommand(() -> {
        //         currentPath = path;
        //         swerveDrive.setDesiredState(SwerveDrive.DesiredState.FOLLOW_PATH);
        //     }),
        //     // Wait for path to complete or use the builder directly
        //     autoBuilder.build(path)
        // );

        return new InstantCommand(() -> robotState.resetPose(new Pose2d(50, 50, new Rotation2d())));
    }
}

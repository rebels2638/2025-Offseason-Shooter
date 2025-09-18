package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.constants.Constants;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.lib.auto.FollowPath;
import frc.robot.lib.auto.Path;


public class RobotContainer {
    public static RobotContainer instance = null;

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final RobotState robotState = RobotState.getInstance(); // we just initialize it here to save time during first reference in subsystems

    private final XboxController xboxTester;
    private final XboxController xboxDriver;
    private final XboxController xboxOperator;

    private final LoggedDashboardChooser<Command> sysidChooser = new LoggedDashboardChooser<>("Auto/SYSIDChooser");

    private RobotContainer() {
        this.xboxTester = new XboxController(1);
        this.xboxOperator = new XboxController(2);
        this.xboxDriver = new XboxController(3);

        swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(xboxDriver));
        xboxDriver.getXButton().onTrue(new InstantCommand(() -> robotState.zeroGyro()));
        FollowPath.setTranslationController(new PIDController(3, 0, 0));
        FollowPath.setRotationController(new PIDController(3, 0, 0));
        FollowPath.setCrossTrackController(new PIDController(3, 0, 0));

        sysidChooser.addOption("DynamicDriveCharacterizationSysIdRoutineForward", swerveDrive.getDynamicDriveCharacterizationSysIdRoutine(Direction.kForward));
        sysidChooser.addOption("DynamicDriveCharacterizationSysIdRoutineReverse", swerveDrive.getDynamicDriveCharacterizationSysIdRoutine(Direction.kReverse));
        sysidChooser.addOption("QuasistaticDriveCharacterizationSysIdRoutineForward", swerveDrive.getQuasistaticDriveCharacterizationSysIdRoutine(Direction.kForward));
        sysidChooser.addOption("QuasistaticDriveCharacterizationSysIdRoutineReverse", swerveDrive.getQuasistaticDriveCharacterizationSysIdRoutine(Direction.kReverse));

        sysidChooser.addOption("DynamicSteerCharacterizationSysIdRoutineForward", swerveDrive.getDynamicSteerCharacterizationSysIdRoutine(Direction.kForward));
        sysidChooser.addOption("DynamicSteerCharacterizationSysIdRoutineReverse", swerveDrive.getDynamicSteerCharacterizationSysIdRoutine(Direction.kReverse));
        sysidChooser.addOption("QuasistaticSteerCharacterizationSysIdRoutineForward", swerveDrive.getQuasistaticSteerCharacterizationSysIdRoutine(Direction.kForward));
        sysidChooser.addOption("QuasistaticSteerCharacterizationSysIdRoutineReverse", swerveDrive.getQuasistaticSteerCharacterizationSysIdRoutine(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        // return sysidChooser.get();

        Path path = new Path("shoptest");

        return  
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> swerveDrive.alignModules(Path.getInitialModuleDirection(path, robotState::getEstimatedPose), 6).get()),
                new FollowPath(
                    path,
                    swerveDrive,
                    robotState::getEstimatedPose,
                    robotState::resetPose,
                    // (Pose2d pose) -> {}, 
                    Constants::shouldFlipPath,
                    robotState::getRobotRelativeSpeeds,
                    swerveDrive::driveRobotRelative
                )
            );

        // return null;
    }

    public void setSwerveCoast(boolean isCoast) {
        swerveDrive.setWheelCoast(isCoast);
    }
    public void stopDrive() {
        swerveDrive.driveFieldRelative(new ChassisSpeeds(0, 0, 0));
    }
}

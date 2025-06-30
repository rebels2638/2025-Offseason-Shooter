package frc.robot;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.FollowPath;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.commands.FollowPath.Waypoint;

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
        return new FollowPath(
            List.of(
                new Waypoint(new Translation2d(5,0), Optional.of(Rotation2d.fromDegrees(180)), Optional.of(Double.valueOf(2))),
                new Waypoint(new Translation2d(5,5), Optional.of(Rotation2d.fromDegrees(270)), Optional.of(Double.valueOf(2))),
                new Waypoint(new Translation2d(0,5), Optional.of(Rotation2d.fromDegrees(0)), Optional.of(Double.valueOf(2))),
                new Waypoint(new Translation2d(0,0), Optional.of(Rotation2d.fromDegrees(180)), Optional.empty())
            )
        );
    }
}

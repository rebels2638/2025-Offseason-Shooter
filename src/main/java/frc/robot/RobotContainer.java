package frc.robot;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RobotContainer {
    public static RobotContainer instance = null;

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private final SwerveDrive swerveDrive;
    private final XboxController xboxTester;
    private final XboxController xboxDriver;
    private final XboxController xboxOperator;

    private final LoggedDashboardChooser<Command> sysidChooser = new LoggedDashboardChooser<>("/Auto/SYSIDChooser");

    private RobotContainer() {
        this.xboxTester = new XboxController(1);
        this.xboxOperator = new XboxController(2);
        this.xboxDriver = new XboxController(3);

        // we just initialize it here to save time during first reference in subsystems
        RobotState.getInstance();

        swerveDrive = SwerveDrive.getInstance();
        swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(xboxDriver));

        sysidChooser.addOption("DynamicDriveCharacterizationSysIdRoutineForward", swerveDrive.getDynamicDriveCharacterizationSysIdRoutine(Direction.kForward));
        sysidChooser.addOption("DynamicDriveCharacterizationSysIdRoutineReverse", swerveDrive.getDynamicDriveCharacterizationSysIdRoutine(Direction.kReverse));
        sysidChooser.addOption("QuasistaticDriveCharacterizationSysIdRoutineForward", swerveDrive.getQuasistaticDriveCharacterizationSysIdRoutine(Direction.kForward));
        sysidChooser.addOption("QuasistaticDriveCharacterizationSysIdRoutineReverse", swerveDrive.getQuasistaticDriveCharacterizationSysIdRoutine(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return sysidChooser.get();
    }
}

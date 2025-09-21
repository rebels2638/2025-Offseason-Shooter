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
import frc.robot.subsystems.turret.Turret;
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
    private final Turret turret = Turret.getInstance();

    private final XboxController xboxTester;
    private final XboxController xboxDriver;
    private final XboxController xboxOperator;


    private RobotContainer() {
        this.xboxTester = new XboxController(1);
        this.xboxOperator = new XboxController(2);
        this.xboxDriver = new XboxController(3);

        swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(xboxDriver));
        xboxDriver.getXButton().onTrue(new InstantCommand(() -> robotState.zeroGyro()));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}

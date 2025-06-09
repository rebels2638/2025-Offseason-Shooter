package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
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


    private RobotContainer() {
        this.xboxTester = new XboxController(1);
        this.xboxOperator = new XboxController(2);
        this.xboxDriver = new XboxController(3);

        // we just initialize it here to save time during first reference in subsystems
        RobotState.getInstance();

        swerveDrive = SwerveDrive.getInstance();
        swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(xboxDriver));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}

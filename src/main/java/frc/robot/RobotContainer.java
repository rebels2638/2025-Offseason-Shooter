package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.input.XboxController;


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


    private RobotContainer() {
        this.xboxTester = new XboxController(1);
        this.xboxOperator = new XboxController(2);
        this.xboxDriver = new XboxController(3);
    }

    public Command getAutonomousCommand() {
        return null;
    }
}

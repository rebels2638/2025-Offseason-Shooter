package frc.robot.constants.robotState;

public class RobotStateConfigSim extends RobotStateConfigBase {
    private static RobotStateConfigSim instance;
    public static RobotStateConfigSim getInstance() {
        if (instance == null) {
            instance = new RobotStateConfigSim();
        }

        return instance;
    }
    
    private RobotStateConfigSim() {}

    @Override
    public double getVisionTranslationDevBase() {
        return 0.02;
    }

    @Override
    public double getVisionRotationDevBase() {
        return 0.02;
    }

    @Override
    public double getOdomTranslationDevBase() {
        return 0.005;
    }

    @Override
    public double getOdomRotationDevBase() {
        return 0.005;
    }
}

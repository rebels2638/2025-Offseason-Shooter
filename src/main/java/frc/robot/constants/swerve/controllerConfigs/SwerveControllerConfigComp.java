package frc.robot.constants.swerve.controllerConfigs;

import edu.wpi.first.math.controller.PIDController;

public class SwerveControllerConfigComp extends SwerveControllerConfigBase {
    
    public static SwerveControllerConfigComp instance = null;
    public static SwerveControllerConfigComp getInstance() {
        if (instance == null) {
            instance = new SwerveControllerConfigComp(0);
        }
        return instance;
    }

    private final PIDController rotationalVelocityFeedbackController;
    private final PIDController translationVelocityFeedbackController;
    private final PIDController rotationalPositionFeedbackController;
    private final double rotationalPositionMaxOutputRadSec;

    /**
     * Constructs a SwerveControllerConfigProto with the specified maximum angular velocity.
     *
     * @param maxAngularVelocityRadiansPerSec The drivetrain's max angular velocity (rad/s)
     */
    private SwerveControllerConfigComp(double maxAngularVelocityRadiansPerSec) {
        // These PIDControllers are zeroed out as specified
        this.rotationalVelocityFeedbackController = new PIDController(0, 0, 0);
        this.translationVelocityFeedbackController = new PIDController(0, 0, 0);
        this.rotationalPositionFeedbackController = new PIDController(2, 0, 0);

        // Rotational position max output is half of the max angular velocity
        this.rotationalPositionMaxOutputRadSec = 0.5 * maxAngularVelocityRadiansPerSec;
    }

    @Override
    public PIDController getRotationalVelocityFeedbackController() {
        return rotationalVelocityFeedbackController;
    }

    @Override
    public PIDController getTranslationVelocityFeedbackController() {
        return translationVelocityFeedbackController;
    }

    @Override
    public PIDController getRotationalPositionFeedbackController() {
        return rotationalPositionFeedbackController;
    }

    @Override
    public double getRotationalPositionMaxOutputRadSec() {
        return rotationalPositionMaxOutputRadSec;
    }
}

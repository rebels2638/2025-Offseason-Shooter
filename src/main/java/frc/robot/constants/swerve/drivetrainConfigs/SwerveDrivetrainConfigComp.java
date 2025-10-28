package frc.robot.constants.swerve.drivetrainConfigs;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.swerve.moduleConfigs.sim.SwerveModuleGeneralConfigSim;

public class SwerveDrivetrainConfigComp extends SwerveDrivetrainConfigBase {

    public static SwerveDrivetrainConfigComp instance = null;

    public static SwerveDrivetrainConfigComp getInstance() {
        if (instance == null) {
            instance = new SwerveDrivetrainConfigComp();
        }

        return instance;
    }

    private SwerveDrivetrainConfigComp() {}
    
    @Override
    public double getMaxTranslationalVelocityMetersPerSec() {
        return getMaxModuleVelocity();
    }

    @Override
    public double getMaxTranslationalAccelerationMetersPerSecSec() {
        return 14;
    }

    @Override
    public double getMaxAngularVelocityRadiansPerSec() {
        return getMaxModuleVelocity() / Math.hypot(getFrontLeftPositionMeters().getX(), getFrontLeftPositionMeters().getY()); 
    }

    @Override
    public double getMaxAngularAccelerationRadiansPerSecSec() {
        return Math.toRadians(2000);
    }

    @Override
    public double getMaxModuleVelocity() {
        return 4.5;
    }

    @Override
    public Translation2d getFrontLeftPositionMeters() {
        return new Translation2d(0.26, 0.26);
    }

    @Override
    public Translation2d getFrontRightPositionMeters() {
        return new Translation2d(0.26, -0.26);
    }

    @Override
    public Translation2d getBackLeftPositionMeters() {
        return new Translation2d(-0.26, 0.26);
    }

    @Override
    public Translation2d getBackRightPositionMeters() {
        return new Translation2d(-0.26, -0.26);
    }

    @Override
    public double getRotationCompensationCoefficient() {
        return 0.0;
    }

    @Override
    public PIDController getTranslationController() {
        PIDController controller = new PIDController(4.0, 0.0, 0.0);
        controller.setTolerance(getTranslationToleranceMeters(), getTranslationVelocityToleranceMeters());
        return controller;
    }

    @Override
    public PIDController getRotationController() {
        PIDController controller = new PIDController(11.0, 0.6, 0.4);
        controller.setIZone(Math.toRadians(10));
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(Math.toRadians(getRotationToleranceDeg()), Math.toRadians(getRotationVelocityToleranceDegPerSec()));

        return controller;
    }

    @Override
    public double getTranslationToleranceMeters() {
        return 0.05;
    }

    @Override
    public double getTranslationVelocityToleranceMeters() {
        return 0.15;
    }

    @Override
    public double getRotationToleranceDeg() {
        return 3.5;
    }

    @Override
    public double getRotationVelocityToleranceDegPerSec() {
        return 5.0;
    }
}
package frc.robot.constants.swerve.drivetrainConfigs;

import java.lang.ModuleLayer.Controller;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.swerve.moduleConfigs.sim.SwerveModuleGeneralConfigSim;

public class SwerveDrivetrainConfigProto extends SwerveDrivetrainConfigBase {

    public static SwerveDrivetrainConfigProto instance = null;
    public static SwerveDrivetrainConfigProto getInstance() {
        if (instance == null) {
            instance = new SwerveDrivetrainConfigProto();
        }
        return instance;
    }

    private SwerveDrivetrainConfigProto() {}

    @Override
    public double getMaxTranslationalVelocityMetersPerSec() {
        return 4.5;
    }

    @Override
    public double getMaxTranslationalAccelerationMetersPerSecSec() {
        return 7;
    }

    @Override
    public double getMaxAngularVelocityRadiansPerSec() {
        return 3.7;
    }

    @Override
    public double getMaxAngularAccelerationRadiansPerSecSec() {
        return 12.0;
    }

    @Override
    public double getMaxModuleVelocity() {
        return 4.5;
    }

    @Override
    public Translation2d getFrontLeftPositionMeters() {
        return new Translation2d(0.38, 0.38);
    }

    @Override
    public Translation2d getFrontRightPositionMeters() {
        return new Translation2d(0.38, -0.38);
    }

    @Override
    public Translation2d getBackLeftPositionMeters() {
        return new Translation2d(-0.38, 0.38);
    }

    @Override
    public Translation2d getBackRightPositionMeters() {
        return new Translation2d(-0.38, -0.38);
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
        PIDController controller = new PIDController(3.0, 0.0, 0.0);
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
        return 10.0;
    }

    @Override
    public double getRotationVelocityToleranceDegPerSec() {
        return 18.0;
    }
}
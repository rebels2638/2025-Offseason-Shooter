package frc.robot.constants.swerve.drivetrainConfigs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class SwerveDrivetrainConfigBase {

    public abstract double getMaxTranslationalVelocityMetersPerSec();

    public abstract double getMaxTranslationalAccelerationMetersPerSecSec();

    public abstract double getMaxAngularVelocityRadiansPerSec();

    public abstract double getMaxAngularAccelerationRadiansPerSecSec();

    public abstract double getMaxModuleVelocity();

    public abstract Translation2d getFrontLeftPositionMeters();

    public abstract Translation2d getFrontRightPositionMeters();

    public abstract Translation2d getBackLeftPositionMeters();

    public abstract Translation2d getBackRightPositionMeters();

    public abstract double getRotationCompensationCoefficient();

    public abstract PIDController getTranslationController();
    public abstract PIDController getRotationController();

    public abstract double getTranslationToleranceMeters();
    public abstract double getTranslationVelocityToleranceMeters();

    public abstract double getRotationToleranceDeg();
    public abstract double getRotationVelocityToleranceDegPerSec();

}

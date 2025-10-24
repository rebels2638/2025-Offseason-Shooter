package frc.robot.constants.swerve.drivetrainConfigs;

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

}

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
}
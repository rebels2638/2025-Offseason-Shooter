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

}
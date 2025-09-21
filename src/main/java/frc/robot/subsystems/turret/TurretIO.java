package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface TurretIO {
    @AutoLog
    class TurretIOInputs {
        public Rotation2d turretPosition = new Rotation2d();
        public double turretVelocityRadPerSec = 0;

        public Rotation2d turretEncoderAbsolutePosition = new Rotation2d();
        public Rotation2d turretEncoderPosition = new Rotation2d();

        public double turretTorqueCurrent = 0;
        public double turretTemperatureFahrenheit = 0;

        public double[] odometryTimestampsSeconds = new double[] {};
        public double[] odometryDrivePositionsMeters = new double[] {};
        public Rotation2d[] odometryTurretPositions = new Rotation2d[] {};
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void setState(SwerveModuleState state) {}
    public default void setTurretTorqueCurrentFOC(double torqueCurrentFOC) {}
}

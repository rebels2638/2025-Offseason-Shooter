package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public double hoodAngleRotations = 0;
        public double hoodVelocityRotationsPerSec = 0;
        public double hoodTorqueCurrent = 0;

        public double flywheelVelocityRotationsPerSec = 0;
        public double flywheelAppliedVolts = 0;
        public double flywheelTorqueCurrent = 0;

        public double feederVelocityRotationsPerSec = 0;
        public double feederAppliedVolts = 0;

        public double indexerVelocityRotationsPerSec = 0;
        public double indexerAppliedVolts = 0;

        public double hoodTemperatureFahrenheit = 0;

        public double flywheelTemperatureFahrenheit = 0;
        public double feederTemperatureFahrenheit = 0;
        public double indexerTemperatureFahrenheit = 0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}
    public default void setAngle(double angleRotations) {}
    public default void setShotVelocity(double velocityRotationsPerSec) {}
    public default void setFeedVelocity(double velocityRotationsPerSec) {}
    public default void setHoodTorqueCurrentFOC(double torqueCurrentFOC) {}
    public default void setFlywheelVoltage(double voltage) {}
    public default void setFeederVoltage(double voltage) {}
    public default void setIndexerVelocity(double velocityRotationsPerSec) {}
    public default void setIndexerVoltage(double voltage) {}

    public default void configureHoodControlLoop(MotorControlLoopConfig config) {}
    public default void configureFlywheelControlLoop(MotorControlLoopConfig config) {}
    public default void configureFeederControlLoop(MotorControlLoopConfig config) {}
    public default void configureIndexerControlLoop(MotorControlLoopConfig config) {}
}

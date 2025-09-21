package frc.robot.constants.turret;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public abstract class TurretConfigBase {
    public abstract String getCanBusName();

    public abstract double getTurretSupplyCurrentLimit();
    public abstract double getTurretSupplyCurrentLimitLowerTime();
    public abstract double getTurretSupplyCurrentLimitLowerLimit();

    public abstract double getTurretStatorCurrentLimit();

    public abstract double getTurretPeakForwardTorqueCurrent();
    public abstract double getTurretPeakReverseTorqueCurrent();

    public abstract double getTurretKS();
    public abstract double getTurretKV();
    public abstract double getTurretKA();
    public abstract double getTurretKP();
    public abstract double getTurretKI();
    public abstract double getTurretKD();

    public abstract double getTurretMotionMagicExpoKA();
    public abstract double getTurretMotionMagicExpoKV();
    public abstract double getTurretMotionMagicCruiseVelocityRotationsPerSec();

    public abstract boolean getIsTurretNeutralModeBrake();

    public abstract double getTurretMotorToOutputShaftRatio();
    public abstract double getTurretRotorToSensorRatio();

    public abstract FeedbackSensorSourceValue getTurretCancoderFeedbackSensorSource();

    public abstract SensorDirectionValue getCancoderSensorDirection();
    public abstract double getCancoderAbsoluteSensorDiscontinuityPoint();

    public abstract int getTurretCanId();

    public abstract boolean getIsTurretInverted();

    public abstract boolean isTurretNeutralModeBrake();
    
    public abstract int getCancoderCanId();

    public abstract double getTurretMaxAngleRad();
    public abstract double getTurretMinAngleRad();
}
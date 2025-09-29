package frc.robot.constants.shooter;

public abstract class ShooterConfigBase {
    public abstract String getCanBusName();

    // Hood motor config (positional control)
    public abstract int getHoodCanId();
    public abstract boolean getIsHoodInverted();

    public abstract double getHoodSupplyCurrentLimit();
    public abstract double getHoodSupplyCurrentLimitLowerTime();
    public abstract double getHoodSupplyCurrentLimitLowerLimit();
    public abstract double getHoodStatorCurrentLimit();
    public abstract double getHoodPeakForwardTorqueCurrent();
    public abstract double getHoodPeakReverseTorqueCurrent();

    public abstract double getHoodKS();
    public abstract double getHoodKV();
    public abstract double getHoodKA();
    public abstract double getHoodKP();
    public abstract double getHoodKI();
    public abstract double getHoodKD();

    public abstract double getHoodMotionMagicCruiseVelocityRotationsPerSec();
    public abstract double getHoodMotionMagicAccelerationRotationsPerSecSec();
    public abstract double getHoodMotionMagicJerkRotationsPerSecSecSec();

    public abstract double getHoodMotorToOutputShaftRatio();

    // Hood angle limits and starting position
    public abstract double getHoodStartingAngleRotations();
    public abstract double getHoodMinAngleRotations();
    public abstract double getHoodMaxAngleRotations();

    // Flywheel motor config (velocity control)
    public abstract int getFlywheelCanId();
    public abstract boolean getIsFlywheelInverted();

    public abstract double getFlywheelSupplyCurrentLimit();
    public abstract double getFlywheelSupplyCurrentLimitLowerTime();
    public abstract double getFlywheelSupplyCurrentLimitLowerLimit();
    public abstract double getFlywheelStatorCurrentLimit();
    public abstract double getFlywheelPeakForwardTorqueCurrent();
    public abstract double getFlywheelPeakReverseTorqueCurrent();

    public abstract double getFlywheelKS();
    public abstract double getFlywheelKV();
    public abstract double getFlywheelKA();
    public abstract double getFlywheelKP();
    public abstract double getFlywheelKI();
    public abstract double getFlywheelKD();

    public abstract double getFlywheelMotionMagicCruiseVelocityRotationsPerSec();
    public abstract double getFlywheelMotionMagicAccelerationRotationsPerSecSec();
    public abstract double getFlywheelMotionMagicJerkRotationsPerSecSecSec();

    public abstract double getFlywheelMotorToOutputShaftRatio();

    // Feeder motor config (velocity control)
    public abstract int getFeederCanId();
    public abstract boolean getIsFeederInverted();

    public abstract double getFeederSupplyCurrentLimit();
    public abstract double getFeederSupplyCurrentLimitLowerTime();
    public abstract double getFeederSupplyCurrentLimitLowerLimit();
    public abstract double getFeederStatorCurrentLimit();
    public abstract double getFeederPeakForwardTorqueCurrent();
    public abstract double getFeederPeakReverseTorqueCurrent();

    public abstract double getFeederKS();
    public abstract double getFeederKV();
    public abstract double getFeederKA();
    public abstract double getFeederKP();
    public abstract double getFeederKI();
    public abstract double getFeederKD();

    public abstract double getFeederMotionMagicCruiseVelocityRotationsPerSec();
    public abstract double getFeederMotionMagicAccelerationRotationsPerSecSec();
    public abstract double getFeederMotionMagicJerkRotationsPerSecSecSec();

    public abstract double getFeederMotorToOutputShaftRatio();
}

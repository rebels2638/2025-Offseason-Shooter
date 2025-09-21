package frc.robot.constants.turret;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class TurretConfigSim extends TurretConfigBase {

    public static TurretConfigSim instance = null;
    public static TurretConfigSim getInstance() {
        if (instance == null) {
            instance = new TurretConfigSim();
        }
        return instance;
    }

    private TurretConfigSim() {}

    @Override
    public String getCanBusName() {
        return "drivetrain";
    }

    @Override
    public double getTurretSupplyCurrentLimit() {
        return 30.0;
    }

    @Override
    public double getTurretSupplyCurrentLimitLowerTime() {
        return 1.5;
    }

    @Override
    public double getTurretSupplyCurrentLimitLowerLimit() {
        return 30.0;
    }

    @Override
    public double getTurretStatorCurrentLimit() {
        return 40.0;
    }

    @Override
    public double getTurretPeakForwardTorqueCurrent() {
        return 40.0;
    }

    @Override
    public double getTurretPeakReverseTorqueCurrent() {
        return -40.0;
    }

    @Override
    public double getTurretKS() {
        return 0.0;
    }

    @Override
    public double getTurretKV() {
        return 0;
    }

    @Override
    public double getTurretKA() {
        return 0.0;
    }

    @Override
    public double getTurretKP() {
        return 4;
    }

    @Override
    public double getTurretKI() {
        return 0.0;
    }

    @Override
    public double getTurretKD() {
        return 0.7;
    }

    @Override
    public double getTurretMotionMagicExpoKA() {
        return 0.25;
    }

    @Override
    public double getTurretMotionMagicExpoKV() {
        return 2.6;
    }

    @Override
    public double getTurretMotionMagicCruiseVelocityRotationsPerSec() {
        return 4;
    }

    @Override
    public boolean getIsTurretNeutralModeBrake() {
        return true;
    }

    @Override
    public double getTurretMotorToOutputShaftRatio() {
        return 21.428;
    }

    @Override
    public double getTurretRotorToSensorRatio() {
        return 21.428;
    }

    @Override
    public FeedbackSensorSourceValue getTurretCancoderFeedbackSensorSource() {
        return FeedbackSensorSourceValue.FusedCANcoder;
    }

    @Override
    public SensorDirectionValue getCancoderSensorDirection() {
        return SensorDirectionValue.CounterClockwise_Positive;
    }

    @Override
    public int getTurretCanId() {
        return 12;
    }

    @Override
    public boolean getIsTurretInverted() {
        return true;
    }

    @Override
    public boolean isTurretNeutralModeBrake() {
        return true;
    }

    @Override
    public int getCancoderCanId() {
        return 13;
    }

    @Override
    public double getCancoderAbsoluteSensorDiscontinuityPoint() {
        return 0.5;
    }

    @Override
    public double getTurretMaxAngleRad() {
        return Math.toRadians(360);
    }

    @Override
    public double getTurretMinAngleRad() {
        return Math.toRadians(-360);
    }
}
package frc.robot.constants.shooter.comp;

import frc.robot.constants.shooter.ShooterConfigBase;

public class ShooterConfigComp extends ShooterConfigBase {

    public static ShooterConfigComp instance = null;
    public static ShooterConfigComp getInstance() {
        if (instance == null) {
            instance = new ShooterConfigComp();
        }
        return instance;
    }

    private ShooterConfigComp() {}

    @Override
    public String getCanBusName() {
        return "rio";
    }

    // Hood motor config
    @Override
    public int getHoodCanId() {
        return 20;
    }

    @Override
    public boolean getIsHoodInverted() {
        return false;
    }

    @Override
    public double getHoodSupplyCurrentLimit() {
        return 30.0;
    }

    @Override
    public double getHoodSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getHoodSupplyCurrentLimitLowerLimit() {
        return 25.0;
    }

    @Override
    public double getHoodStatorCurrentLimit() {
        return 40.0;
    }

    @Override
    public double getHoodPeakForwardTorqueCurrent() {
        return 40.0;
    }

    @Override
    public double getHoodPeakReverseTorqueCurrent() {
        return -40.0;
    }

    @Override
    public double getHoodKS() {
        return 0.1;
    }

    @Override
    public double getHoodKV() {
        return 0.12;
    }

    @Override
    public double getHoodKA() {
        return 0.01;
    }

    @Override
    public double getHoodKP() {
        return 50.0;
    }

    @Override
    public double getHoodKI() {
        return 0.0;
    }

    @Override
    public double getHoodKD() {
        return 5.0;
    }

    @Override
    public double getHoodMotionMagicCruiseVelocityRotationsPerSec() {
        return 10.0;
    }

    @Override
    public double getHoodMotionMagicAccelerationRotationsPerSecSec() {
        return 20.0;
    }

    @Override
    public double getHoodMotionMagicJerkRotationsPerSecSecSec() {
        return 100.0;
    }

    @Override
    public double getHoodMotorToOutputShaftRatio() {
        return 100.0; // 100:1 gear ratio
    }

    @Override
    public double getHoodStartingAngleRotations() {
        return 0.0;
    }

    @Override
    public double getHoodMinAngleRotations() {
        return -0.5;
    }

    @Override
    public double getHoodMaxAngleRotations() {
        return 0.5;
    }

    // Flywheel motor config
    @Override
    public int getFlywheelCanId() {
        return 21;
    }

    @Override
    public boolean getIsFlywheelInverted() {
        return false;
    }

    @Override
    public double getFlywheelSupplyCurrentLimit() {
        return 60.0;
    }

    @Override
    public double getFlywheelSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getFlywheelSupplyCurrentLimitLowerLimit() {
        return 50.0;
    }

    @Override
    public double getFlywheelStatorCurrentLimit() {
        return 80.0;
    }

    @Override
    public double getFlywheelPeakForwardTorqueCurrent() {
        return 80.0;
    }

    @Override
    public double getFlywheelPeakReverseTorqueCurrent() {
        return -80.0;
    }

    @Override
    public double getFlywheelKS() {
        return 0.1;
    }

    @Override
    public double getFlywheelKV() {
        return 0.002;
    }

    @Override
    public double getFlywheelKA() {
        return 0.0005;
    }

    @Override
    public double getFlywheelKP() {
        return 0.1;
    }

    @Override
    public double getFlywheelKI() {
        return 0.0;
    }

    @Override
    public double getFlywheelKD() {
        return 0.0;
    }

    @Override
    public double getFlywheelMotionMagicCruiseVelocityRotationsPerSec() {
        return 100.0;
    }

    @Override
    public double getFlywheelMotionMagicAccelerationRotationsPerSecSec() {
        return 200.0;
    }

    @Override
    public double getFlywheelMotionMagicJerkRotationsPerSecSecSec() {
        return 1000.0;
    }

    @Override
    public double getFlywheelMotorToOutputShaftRatio() {
        return 1.0; // Direct drive
    }

    // Feeder motor config
    @Override
    public int getFeederCanId() {
        return 22;
    }

    @Override
    public boolean getIsFeederInverted() {
        return false;
    }

    @Override
    public double getFeederSupplyCurrentLimit() {
        return 30.0;
    }

    @Override
    public double getFeederSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getFeederSupplyCurrentLimitLowerLimit() {
        return 25.0;
    }

    @Override
    public double getFeederStatorCurrentLimit() {
        return 40.0;
    }

    @Override
    public double getFeederPeakForwardTorqueCurrent() {
        return 40.0;
    }

    @Override
    public double getFeederPeakReverseTorqueCurrent() {
        return -40.0;
    }

    @Override
    public double getFeederKS() {
        return 0.1;
    }

    @Override
    public double getFeederKV() {
        return 0.002;
    }

    @Override
    public double getFeederKA() {
        return 0.0005;
    }

    @Override
    public double getFeederKP() {
        return 0.1;
    }

    @Override
    public double getFeederKI() {
        return 0.0;
    }

    @Override
    public double getFeederKD() {
        return 0.0;
    }

    @Override
    public double getFeederMotionMagicCruiseVelocityRotationsPerSec() {
        return 50.0;
    }

    @Override
    public double getFeederMotionMagicAccelerationRotationsPerSecSec() {
        return 100.0;
    }

    @Override
    public double getFeederMotionMagicJerkRotationsPerSecSecSec() {
        return 500.0;
    }

    @Override
    public double getFeederMotorToOutputShaftRatio() {
        return 5.0; // 5:1 gear ratio
    }
}

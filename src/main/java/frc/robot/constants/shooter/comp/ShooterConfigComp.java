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
        return "drivetrain";
    }

    // Hood motor config
    @Override
    public int getHoodCanId() {
        return 19;
    }

    @Override
    public boolean getIsHoodInverted() {
        return false;
    }

    @Override
    public double getHoodSupplyCurrentLimit() {
        return 80.0;
    }

    @Override
    public double getHoodSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getHoodSupplyCurrentLimitLowerLimit() {
        return 80.0;
    }

    @Override
    public double getHoodStatorCurrentLimit() {
        return 80.0;
    }

    @Override
    public double getHoodPeakForwardTorqueCurrent() {
        return 80.0;
    }

    @Override
    public double getHoodPeakReverseTorqueCurrent() {
        return -80.0;
    }

    @Override
    public double getHoodKS() {
        return 0;
    }

    @Override
    public double getHoodKV() {
        return 0;
    }

    @Override
    public double getHoodKA() {
        return 0.0;
    }

    @Override
    public double getHoodKP() {
        return 50; // 300
    }

    @Override
    public double getHoodKI() {
        return 0.0; // 10
    }

    @Override
    public double getHoodKD() {
        return 1.0; // 12
    }

    @Override
    public double getHoodMotionMagicCruiseVelocityRotationsPerSec() {
        return 4.0;
    }

    @Override
    public double getHoodMotionMagicAccelerationRotationsPerSecSec() {
        return 11.0;
    }

    @Override
    public double getHoodMotionMagicJerkRotationsPerSecSecSec() {
        return 40.0;
    }

    @Override
    public double getHoodMotorToOutputShaftRatio() {
        return 84.0/36.0; 
    }

    @Override
    public double getHoodStartingAngleRotations() {
        return 36.0/360.0;
    }

    @Override
    public double getHoodMinAngleRotations() {
        return 36.0/360.0;
    }

    @Override
    public double getHoodMaxAngleRotations() {
        return 150.0/360.0;
    }

    // Flywheel motor config
    @Override
    public int getFlywheelCanId() {
        return 16;
    }

    @Override
    public boolean getIsFlywheelInverted() {
        return false;
    }

    @Override
    public double getFlywheelSupplyCurrentLimit() {
        return 15.0;
    }

    @Override
    public double getFlywheelSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getFlywheelSupplyCurrentLimitLowerLimit() {
        return 15.0;
    }

    @Override
    public double getFlywheelStatorCurrentLimit() {
        return 15.0;
    }

    @Override
    public double getFlywheelPeakForwardTorqueCurrent() {
        return 15.0;
    }

    @Override
    public double getFlywheelPeakReverseTorqueCurrent() {
        return -15.0;
    }

    @Override
    public double getFlywheelKS() {
        return 0.1;
    }

    @Override
    public double getFlywheelKV() {
        return 0.188;
    }

    @Override
    public double getFlywheelKA() {
        return 0.0;
    }

    @Override
    public double getFlywheelKP() {
        return 0.2;
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
        return 18.0/12.0; // Direct drive
    }

    // Feeder motor config
    @Override
    public int getFeederCanId() {
        return 35;
    }

    @Override
    public boolean getIsFeederInverted() {
        return false;
    }

    @Override
    public double getFeederSupplyCurrentLimit() {
        return 90.0;
    }

    @Override
    public double getFeederSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getFeederSupplyCurrentLimitLowerLimit() {
        return 90.0;
    }

    @Override
    public double getFeederStatorCurrentLimit() {
        return 90.0;
    }

    @Override
    public double getFeederPeakForwardTorqueCurrent() {
        return 90.0;
    }

    @Override
    public double getFeederPeakReverseTorqueCurrent() {
        return -90.0;
    }

    @Override
    public double getFeederKS() {
        return 0.0;
    }

    @Override
    public double getFeederKV() {
        return 0.00;
    }

    @Override
    public double getFeederKA() {
        return 0.000;
    }

    @Override
    public double getFeederKP() {
        return 0;
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
        return 3.0; // 5:1 gear ratio
    }

    // Indexer motor config
    @Override
    public int getIndexerCanId() {
        return 20;
    }

    @Override
    public boolean getIsIndexerInverted() {
        return true;
    }

    @Override
    public double getIndexerSupplyCurrentLimit() {
        return 90.0;
    }

    @Override
    public double getIndexerSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getIndexerSupplyCurrentLimitLowerLimit() {
        return 90.0;
    }

    @Override
    public double getIndexerStatorCurrentLimit() {
        return 90.0;
    }

    @Override
    public double getIndexerPeakForwardTorqueCurrent() {
        return 90.0;
    }

    @Override
    public double getIndexerPeakReverseTorqueCurrent() {
        return -90.0;
    }

    @Override
    public double getIndexerKS() {
        return 0.0;
    }

    @Override
    public double getIndexerKV() {
        return 0.00;
    }

    @Override
    public double getIndexerKA() {
        return 0.000;
    }

    @Override
    public double getIndexerKP() {
        return 0;
    }

    @Override
    public double getIndexerKI() {
        return 0.0;
    }

    @Override
    public double getIndexerKD() {
        return 0.0;
    }

    @Override
    public double getIndexerMotionMagicCruiseVelocityRotationsPerSec() {
        return 50.0;
    }

    @Override
    public double getIndexerMotionMagicAccelerationRotationsPerSecSec() {
        return 100.0;
    }

    @Override
    public double getIndexerMotionMagicJerkRotationsPerSecSecSec() {
        return 500.0;
    }

    @Override
    public double getIndexerMotorToOutputShaftRatio() {
        return 3.0; // 5:1 gear ratio
    }
}

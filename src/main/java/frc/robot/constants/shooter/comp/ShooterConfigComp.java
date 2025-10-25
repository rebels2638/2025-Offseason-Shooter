package frc.robot.constants.shooter.comp;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
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
    public InterpolatingMatrixTreeMap<Double, N2, N1> getLerpTable() {
        InterpolatingMatrixTreeMap<Double, N2, N1> table = new InterpolatingMatrixTreeMap<>();
        table.put(0.2, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            125.0/360.0, // hood angle in rotations
            2.0 // flywheel velocity in rotations per second
        }));
        table.put(0.4, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            125.0/360.0, // hood angle in rotations
            2.0 // flywheel velocity in rotations per second
        }));
        table.put(0.6, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            125.0/360.0, // hood angle in rotations
            2.0 // flywheel velocity in rotations per second
        }));
        table.put(0.8, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            125.0/360.0, // hood angle in rotations
            2.0 // flywheel velocity in rotations per second
        }));
        table.put(1.0, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            125.0/360.0, // hood angle in rotations
            2.0 // flywheel velocity in rotations per second
        }));
        table.put(1.2, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            125.0/360.0, // hood angle in rotations
            2.0 // flywheel velocity in rotations per second
        }));
        table.put(1.4, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            125.0/360.0, // hood angle in rotations
            2.0 // flywheel velocity in rotations per second
        }));
        table.put(1.6, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            125.0/360.0, // hood angle in rotations
            2.0 // flywheel velocity in rotations per second
        }));
        table.put(1.8, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            125.0/360.0, // hood angle in rotations
            2.0 // flywheel velocity in rotations per second
        }));
        return table;
    }

    @Override
    public double getMinShotDistFromShooterMeters() {
        return 0.2;
    }

    @Override
    public double getMaxShotDistFromShooterMeters() {
        return 1.8;
    }

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
        return true;
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
    public double getHoodMotorToOutputShaftRatio() {
        return 84.0/36.0; 
    }

    // shot exit angle above horizontal
    @Override
    public double getHoodStartingAngleRotations() {
        return 54.0/360.0;
    }

    @Override
    public double getHoodMinAngleRotations() {
        return -120/360.0;
    }

    @Override
    public double getHoodMaxAngleRotations() {
        return 54.0/360.0; // 150
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
    public double getFlywheelMotorToOutputShaftRatio() {
        return 18.0/12.0; // Direct drive
    }

    @Override
    public double getFlywheelRadiusMeters() {
        return 0.0508; // 2 inches in meters
    }

    @Override
    public Pose3d getShooterPose3d() {
        // Shooter position relative to robot center
        // X: forward from robot center (meters)
        // Y: left/right offset (meters, positive = left)
        // Z: height above ground (meters)
        return new Pose3d(
            new Translation3d(0.25, 0.0, 0.2),
            new Rotation3d(0.0, 0.0, 0.0)
        );
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
        return 0.3;
    }

    @Override
    public double getFeederKA() {
        return 0.000;
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
        return 0.3;
    }

    @Override
    public double getIndexerKA() {
        return 0.0;
    }

    @Override
    public double getIndexerKP() {
        return 0.1;
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
    public double getIndexerMotorToOutputShaftRatio() {
        return 3.0; // 5:1 gear ratio
    }

    @Override
    public double getHoodAngleToleranceRotations() {
        return 5.0 / 360.0;
    }

    @Override
    public double getFlywheelVelocityToleranceRPS() {
        return 3.0;
    }

    @Override
    public double getFeederVelocityToleranceRPS() {
        return 5.0;
    }

    @Override
    public double getIndexerVelocityToleranceRPS() {
        return 5.0;
    }
}

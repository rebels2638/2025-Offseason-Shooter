package frc.robot.constants.shooter.sim;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import frc.robot.constants.shooter.ShooterConfigBase;

public class ShooterConfigSim extends ShooterConfigBase {

    public static ShooterConfigSim instance = null;
    public static ShooterConfigSim getInstance() {
        if (instance == null) {
            instance = new ShooterConfigSim();
        }
        return instance;
    }

    private ShooterConfigSim() {}
    
    @Override
    public InterpolatingMatrixTreeMap<Double, N2, N1> getLerpTable() {
        // Calculated using projectile motion with:
        // - Flywheel radius: 0.0508m (2 inches)
        // - Shooter height: 0.2m above ground
        // - Exit velocity = flywheelRPS * π * radius
        // - Hood angle relative to horizontal
        InterpolatingMatrixTreeMap<Double, N2, N1> table = new InterpolatingMatrixTreeMap<>();
        
        table.put(0.5, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.08888889, // hood angle = 32.00° (exit velocity = 1.81 m/s)
            11.37 // flywheel velocity in RPS
        }));
        
        table.put(1.0, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.09444444, // hood angle = 34.00° (exit velocity = 2.84 m/s)
            17.82 // flywheel velocity in RPS
        }));
        
        table.put(1.5, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.11111111, // hood angle = 40.00° (exit velocity = 3.58 m/s)
            22.43 // flywheel velocity in RPS
        }));
        
        table.put(2.0, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.11388889, // hood angle = 41.00° (exit velocity = 4.21 m/s)
            26.35 // flywheel velocity in RPS
        }));
        
        table.put(2.5, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.11388889, // hood angle = 41.00° (exit velocity = 4.76 m/s)
            29.81 // flywheel velocity in RPS
        }));
        
        table.put(3.0, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.11111111, // hood angle = 40.00° (exit velocity = 5.25 m/s)
            32.92 // flywheel velocity in RPS
        }));
        
        table.put(3.5, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.11666667, // hood angle = 42.00° (exit velocity = 5.70 m/s)
            35.69 // flywheel velocity in RPS
        }));
        
        table.put(4.0, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.12222222, // hood angle = 44.00° (exit velocity = 6.11 m/s)
            38.28 // flywheel velocity in RPS
        }));
        
        table.put(4.5, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.11666667, // hood angle = 42.00° (exit velocity = 6.50 m/s)
            40.76 // flywheel velocity in RPS
        }));
        
        table.put(5.0, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.12222222, // hood angle = 44.00° (exit velocity = 6.86 m/s)
            43.00 // flywheel velocity in RPS
        }));
        
        table.put(5.5, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.12222222, // hood angle = 44.00° (exit velocity = 7.21 m/s)
            45.19 // flywheel velocity in RPS
        }));
        
        table.put(6.0, new Matrix<N2, N1>(Nat.N2(), Nat.N1(), new double[]{
            0.12222222, // hood angle = 44.00° (exit velocity = 7.54 m/s)
            47.27 // flywheel velocity in RPS
        }));
        
        return table;
    }

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
        return 25.0;
    }

    @Override
    public double getHoodKI() {
        return 0.0;
    }

    @Override
    public double getHoodKD() {
        return 0.0;
    }

    @Override
    public double getHoodMotorToOutputShaftRatio() {
        return 100.0;
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
        return 0;
    }

    @Override
    public double getFlywheelKV() {
        return 0.03;
    }

    @Override
    public double getFlywheelKA() {
        return 0;
    }

    @Override
    public double getFlywheelKP() {
        return 0.02;
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
        return 1.0;
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
        return 0;
    }

    @Override
    public double getFeederKV() {
        return 0.03;
    }

    @Override
    public double getFeederKA() {
        return 0;
    }

    @Override
    public double getFeederKP() {
        return 0.02;
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
        return 5.0;
    }

    // Indexer motor config
    @Override
    public int getIndexerCanId() {
        return 23;
    }

    @Override
    public boolean getIsIndexerInverted() {
        return false;
    }

    @Override
    public double getIndexerSupplyCurrentLimit() {
        return 30.0;
    }

    @Override
    public double getIndexerSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getIndexerSupplyCurrentLimitLowerLimit() {
        return 25.0;
    }

    @Override
    public double getIndexerStatorCurrentLimit() {
        return 40.0;
    }

    @Override
    public double getIndexerPeakForwardTorqueCurrent() {
        return 40.0;
    }

    @Override
    public double getIndexerPeakReverseTorqueCurrent() {
        return -40.0;
    }

    @Override
    public double getIndexerKS() {
        return 0;
    }

    @Override
    public double getIndexerKV() {
        return 0.03;
    }

    @Override
    public double getIndexerKA() {
        return 0;
    }

    @Override
    public double getIndexerKP() {
        return 0.02;
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
        return 5.0;
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

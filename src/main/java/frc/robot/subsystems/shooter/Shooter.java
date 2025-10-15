package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.shooter.ShooterConfigBase;
import frc.robot.constants.shooter.comp.ShooterConfigComp;
import frc.robot.constants.shooter.proto.ShooterConfigProto;
import frc.robot.constants.shooter.sim.ShooterConfigSim;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator;

public class Shooter extends SubsystemBase {
    private static Shooter instance = null;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    private final ShooterConfigBase config;
    
    private final DashboardMotorControlLoopConfigurator hoodControlLoopConfigurator;
    private final DashboardMotorControlLoopConfigurator flywheelControlLoopConfigurator;
    private final DashboardMotorControlLoopConfigurator feederControlLoopConfigurator;
    private final DashboardMotorControlLoopConfigurator indexerControlLoopConfigurator;

    private Shooter() {
        switch (Constants.currentMode) {
            case COMP:
                config = ShooterConfigComp.getInstance();
                shooterIO = new ShooterIOTalonFX(config);
                break;

            case PROTO:
                config = ShooterConfigProto.getInstance();
                shooterIO = new ShooterIOTalonFX(config);
                break;

            case SIM:
                config = ShooterConfigSim.getInstance();
                shooterIO = new ShooterIOSim(config);
                break;

            case REPLAY:
                config = ShooterConfigComp.getInstance();
                shooterIO = new ShooterIOTalonFX(config);
                break;

            default:
                config = ShooterConfigComp.getInstance();
                shooterIO = new ShooterIOTalonFX(config);
                break;
        }

        hoodControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Shooter/hoodControlLoop", 
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.getHoodKP(),
                config.getHoodKI(),
                config.getHoodKD(),
                config.getHoodKS(),
                config.getHoodKV(),
                config.getHoodKA()
            )
        );
        flywheelControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Shooter/flywheelControlLoop", 
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.getFlywheelKP(),
                config.getFlywheelKI(),
                config.getFlywheelKD(),
                config.getFlywheelKS(),
                config.getFlywheelKV(),
                config.getFlywheelKA()
            )
        );
        feederControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Shooter/feederControlLoop", 
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.getFeederKP(),
                config.getFeederKI(),
                config.getFeederKD(),
                config.getFeederKS(),
                config.getFeederKV(),
                config.getFeederKA()
            )
        );
        indexerControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Shooter/indexerControlLoop", 
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.getIndexerKP(),
                config.getIndexerKI(),
                config.getIndexerKD(),
                config.getIndexerKS(),
                config.getIndexerKV(),
                config.getIndexerKA()
            )
        );
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);

        if (hoodControlLoopConfigurator.hasChanged()) {
            shooterIO.configureHoodControlLoop(hoodControlLoopConfigurator.getConfig());
        }
        if (flywheelControlLoopConfigurator.hasChanged()) {
            shooterIO.configureFlywheelControlLoop(flywheelControlLoopConfigurator.getConfig());
        }
        if (feederControlLoopConfigurator.hasChanged()) {
            shooterIO.configureFeederControlLoop(feederControlLoopConfigurator.getConfig());
        }
        if (indexerControlLoopConfigurator.hasChanged()) {
            shooterIO.configureIndexerControlLoop(indexerControlLoopConfigurator.getConfig());
        }
    }

    public void setAngle(Rotation2d angle) {
        double clampedAngle = Math.max(config.getHoodMinAngleRotations(),
        Math.min(config.getHoodMaxAngleRotations(), angle.getRotations()));        

        Logger.recordOutput("Shooter/angleSetpointRotations", clampedAngle);
        shooterIO.setAngle(clampedAngle);
    }

    public void setShotVelocity(double velocityRotationsPerSec) {
        Logger.recordOutput("Shooter/shotVelocitySetpointRotationsPerSec", velocityRotationsPerSec);
        shooterIO.setShotVelocity(velocityRotationsPerSec);
    }

    public void setFeedVelocity(double velocityRotationsPerSec) {
        Logger.recordOutput("Shooter/feedVelocitySetpointRotationsPerSec", velocityRotationsPerSec);
        shooterIO.setFeedVelocity(velocityRotationsPerSec);
    }

    public void setIndexerVelocity(double velocityRotationsPerSec) {
        Logger.recordOutput("Shooter/indexerVelocitySetpointRotationsPerSec", velocityRotationsPerSec);
        shooterIO.setIndexerVelocity(velocityRotationsPerSec);
    }

    public double getHoodAngleRotations() {
        return shooterInputs.hoodAngleRotations;
    }

    public double getFlywheelVelocityRotationsPerSec() {
        return shooterInputs.flywheelVelocityRotationsPerSec;
    }

    public double getFeederVelocityRotationsPerSec() {
        return shooterInputs.feederVelocityRotationsPerSec;
    }

    public double getIndexerVelocityRotationsPerSec() {
        return shooterInputs.indexerVelocityRotationsPerSec;
    }
}

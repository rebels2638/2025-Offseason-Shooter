package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.shooter.ShooterConfigBase;
import frc.robot.constants.shooter.comp.ShooterConfigComp;
import frc.robot.constants.shooter.proto.ShooterConfigProto;
import frc.robot.constants.shooter.sim.ShooterConfigSim;

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
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);
    }

    public void setAngle(double angleRotations) {
        shooterIO.setAngle(angleRotations);
    }

    public void setShotVelocity(double velocityRotationsPerSec) {
        shooterIO.setShotVelocity(velocityRotationsPerSec);
    }

    public void setFeedVelocity(double velocityRotationsPerSec) {
        shooterIO.setFeedVelocity(velocityRotationsPerSec);
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
}

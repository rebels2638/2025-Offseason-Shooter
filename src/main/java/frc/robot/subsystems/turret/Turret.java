package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.turret.TurretConfigBase;
import frc.robot.constants.turret.TurretConfigComp;
import frc.robot.constants.turret.TurretConfigProto;
import frc.robot.constants.turret.TurretConfigSim;

public class Turret extends SubsystemBase {
    private static Turret instance = null;
    public static Turret getInstance() {
        if (instance == null) {
            instance = new Turret();
        }
        return instance;
    }

    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private TurretConfigBase config;

    private Rotation2d setpoint = new Rotation2d();

    private Turret() {
        switch (Constants.currentMode) {
            case SIM:
                config = TurretConfigSim.getInstance();
                turretIO = new TurretIOSim(config);
                break;
            case COMP:
                config = TurretConfigComp.getInstance();
                turretIO = new TurretIOTalonFX(config);
                break;
            case PROTO:
                config = TurretConfigProto.getInstance();
                turretIO = new TurretIOTalonFX(config);
                break;
            default:
                config = TurretConfigComp.getInstance();
                turretIO = new TurretIOTalonFX(config);
                break;
        }

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void periodic() {
        turretIO.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        turretIO.setState(new SwerveModuleState(0, setpoint));
        Logger.recordOutput("Turret/currentAngle", inputs.turretPosition.getDegrees()); //this is for my own sanity LOL
    }

    public void setTurretAngle(Rotation2d angle) {
        setpoint = new Rotation2d(MathUtil.clamp(angle.getRadians(), config.getTurretMinAngleRad(), config.getTurretMaxAngleRad()));
        Logger.recordOutput("Turret/setpointDeg", setpoint.getDegrees());
    }

    public void setTorqueCurrentFOC(double torque) {
        turretIO.setTurretTorqueCurrentFOC(torque);
    }

    public Rotation2d getAngularPosition() {
        return inputs.turretPosition;
    }

    public boolean reachedMaxRotation() {
        return Math.abs(inputs.turretPosition.getDegrees()) >= 360.0;
    }
}
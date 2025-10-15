package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.shooter.ShooterConfigBase;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;

public class ShooterIOSim implements ShooterIO {
    private final DCMotor hoodMotorModel = DCMotor.getKrakenX60Foc(1);
    private final DCMotor flywheelMotorModel = DCMotor.getKrakenX60Foc(1);
    private final DCMotor feederMotorModel = DCMotor.getKrakenX60Foc(1);
    private final DCMotor indexerMotorModel = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim hoodSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(hoodMotorModel, 0.00015, 21.428), // magic number because hood is not important
            hoodMotorModel
        );
    private final DCMotorSim flywheelSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(flywheelMotorModel, (2.8087 * 0.0194 * 0.0485614385) / 6.12, 6.12),
            flywheelMotorModel
        );
    private final DCMotorSim feederSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(feederMotorModel, (2.8087 * 0.0194 * 0.0485614385) / 6.12, 6.12),
            feederMotorModel
        );
    private final DCMotorSim indexerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(indexerMotorModel, (2.8087 * 0.0194 * 0.0485614385) / 6.12, 6.12),
            indexerMotorModel
        );

    private ProfiledPIDController hoodFeedback;
    private PIDController flywheelFeedback;
    private PIDController feederFeedback;
    private PIDController indexerFeedback;

    private SimpleMotorFeedforward flywheelFeedforward;
    private SimpleMotorFeedforward feederFeedforward;
    private SimpleMotorFeedforward indexerFeedforward;

    private boolean isHoodClosedLoop = true;
    private boolean isFlywheelClosedLoop = true;
    private boolean isFeederClosedLoop = true;
    private boolean isIndexerClosedLoop = true;

    private double desiredFlywheelVelocityRotationsPerSec = 0;
    private double desiredFeederVelocityRotationsPerSec = 0;
    private double desiredIndexerVelocityRotationsPerSec = 0;

    private double lastTimeInputs = Timer.getTimestamp();

    private final ShooterConfigBase config;

    public ShooterIOSim(ShooterConfigBase config) {
        this.config = config;

        // Initialize PID controllers with config values
        hoodFeedback = new ProfiledPIDController(config.getHoodKP(), config.getHoodKI(), config.getHoodKD(), 
            new TrapezoidProfile.Constraints(config.getHoodMotionMagicCruiseVelocityRotationsPerSec(), config.getHoodMotionMagicAccelerationRotationsPerSecSec())
        );
        flywheelFeedback = new PIDController(config.getFlywheelKP(), config.getFlywheelKI(), config.getFlywheelKD());
        feederFeedback = new PIDController(config.getFeederKP(), config.getFeederKI(), config.getFeederKD());
        indexerFeedback = new PIDController(config.getIndexerKP(), config.getIndexerKI(), config.getIndexerKD());

        // Initialize feedforward controllers with config values
        flywheelFeedforward = new SimpleMotorFeedforward(config.getFlywheelKS(), config.getFlywheelKV(), config.getFlywheelKA());
        feederFeedforward = new SimpleMotorFeedforward(config.getFeederKS(), config.getFeederKV(), config.getFeederKA());
        indexerFeedforward = new SimpleMotorFeedforward(config.getIndexerKS(), config.getIndexerKV(), config.getIndexerKA());

        hoodFeedback.enableContinuousInput(-Math.PI, Math.PI);

        // Initialize hood position to starting angle
        hoodSim.setState(config.getHoodStartingAngleRotations() * 2 * Math.PI, 0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        double dt = Timer.getTimestamp() - lastTimeInputs;
        lastTimeInputs = Timer.getTimestamp();

        if (isHoodClosedLoop) {
            hoodSim.setInputVoltage(
                MathUtil.clamp(
                    hoodFeedback.calculate(hoodSim.getAngularPositionRad()),
                    -12,
                    12
                )
            );
        }

        if (isFlywheelClosedLoop) {
            flywheelSim.setInputVoltage(
                MathUtil.clamp(
                    flywheelFeedforward.calculate(desiredFlywheelVelocityRotationsPerSec) +
                    flywheelFeedback.calculate(flywheelSim.getAngularVelocityRadPerSec() * 0.0485614385),
                    -12,
                    12
                )
            );
        }

        if (isFeederClosedLoop) {
            feederSim.setInputVoltage(
                MathUtil.clamp(
                    feederFeedforward.calculate(desiredFeederVelocityRotationsPerSec) +
                    feederFeedback.calculate(feederSim.getAngularVelocityRadPerSec() * 0.0485614385),
                    -12,
                    12
                )
            );
        }

        if (isIndexerClosedLoop) {
            indexerSim.setInputVoltage(
                MathUtil.clamp(
                    indexerFeedforward.calculate(desiredIndexerVelocityRotationsPerSec) +
                    indexerFeedback.calculate(indexerSim.getAngularVelocityRadPerSec() * 0.0485614385),
                    -12,
                    12
                )
            );
        }

        hoodSim.update(dt);
        flywheelSim.update(dt);
        feederSim.update(dt);
        indexerSim.update(dt);

        inputs.hoodAngleRotations = hoodSim.getAngularPositionRotations();
        inputs.hoodVelocityRotationsPerSec = hoodSim.getAngularVelocityRadPerSec();

        inputs.flywheelVelocityRotationsPerSec = flywheelSim.getAngularVelocityRadPerSec() * 0.0485614385;
        inputs.flywheelAppliedVolts = flywheelSim.getInputVoltage();

        inputs.feederVelocityRotationsPerSec = feederSim.getAngularVelocityRadPerSec() * 0.0485614385;
        inputs.feederAppliedVolts = feederSim.getInputVoltage();

        inputs.indexerVelocityRotationsPerSec = indexerSim.getAngularVelocityRadPerSec() * 0.0485614385;
        inputs.indexerAppliedVolts = indexerSim.getInputVoltage();

        inputs.hoodTorqueCurrent = hoodSim.getCurrentDrawAmps();

        // Simulation doesn't have temperature sensors, use default values
        inputs.hoodTemperatureFahrenheit = 70.0;
        inputs.flywheelTemperatureFahrenheit = 70.0;
        inputs.feederTemperatureFahrenheit = 70.0;
        inputs.indexerTemperatureFahrenheit = 70.0;
    }

    @Override
    public void setAngle(double angleRotations) {
        // Clamp angle within software limits
        double clampedAngle = MathUtil.clamp(angleRotations,
            config.getHoodMinAngleRotations(),
            config.getHoodMaxAngleRotations());
        hoodFeedback.setGoal(clampedAngle * (2 * Math.PI));
        isHoodClosedLoop = true;
    }

    @Override
    public void setShotVelocity(double velocityRotationsPerSec) {
        flywheelFeedback.setSetpoint(velocityRotationsPerSec);
        desiredFlywheelVelocityRotationsPerSec = velocityRotationsPerSec;
        isFlywheelClosedLoop = true;
    }

    @Override
    public void setFeedVelocity(double velocityRotationsPerSec) {
        feederFeedback.setSetpoint(velocityRotationsPerSec);
        desiredFeederVelocityRotationsPerSec = velocityRotationsPerSec;
        isFeederClosedLoop = true;
    }

    @Override
    public void setHoodTorqueCurrentFOC(double torqueCurrentFOC) {
        hoodSim.setInputVoltage(torqueCurrentFOC);
        isHoodClosedLoop = false;
    }

    @Override
    public void setFlywheelVoltage(double voltage) {
        flywheelSim.setInputVoltage(voltage);
        isFlywheelClosedLoop = false;
    }
    
    @Override
    public void setFeederVoltage(double voltage) {
        feederSim.setInputVoltage(voltage);
        isFeederClosedLoop = false;
    }

    @Override
    public void setIndexerVelocity(double velocityRotationsPerSec) {
        indexerFeedback.setSetpoint(velocityRotationsPerSec);
        desiredIndexerVelocityRotationsPerSec = velocityRotationsPerSec;
        isIndexerClosedLoop = true;
    }

    @Override
    public void setIndexerVoltage(double voltage) {
        indexerSim.setInputVoltage(voltage);
        isIndexerClosedLoop = false;
    }

    @Override
    public void configureHoodControlLoop(MotorControlLoopConfig config) {
        hoodFeedback.setP(config.kP());
        hoodFeedback.setI(config.kI());
        hoodFeedback.setD(config.kD());
    }

    @Override
    public void configureFlywheelControlLoop(MotorControlLoopConfig config) {
        flywheelFeedback.setP(config.kP());
        flywheelFeedback.setI(config.kI());
        flywheelFeedback.setD(config.kD());

        flywheelFeedforward.setKs(config.kS());
        flywheelFeedforward.setKv(config.kV());
        flywheelFeedforward.setKa(config.kA());
    }
    
    @Override
    public void configureFeederControlLoop(MotorControlLoopConfig config) {
        feederFeedback.setP(config.kP());
        feederFeedback.setI(config.kI());
        feederFeedback.setD(config.kD());

        feederFeedforward.setKs(config.kS());
        feederFeedforward.setKv(config.kV());
        feederFeedforward.setKa(config.kA());
    }

    @Override
    public void configureIndexerControlLoop(MotorControlLoopConfig config) {
        indexerFeedback.setP(config.kP());
        indexerFeedback.setI(config.kI());
        indexerFeedback.setD(config.kD());

        indexerFeedforward.setKs(config.kS());
        indexerFeedforward.setKv(config.kV());
        indexerFeedforward.setKa(config.kA());
    }
}

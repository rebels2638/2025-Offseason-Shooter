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

public class ShooterIOSim implements ShooterIO {
    private final DCMotor hoodMotorModel = DCMotor.getKrakenX60Foc(1);
    private final DCMotor flywheelMotorModel = DCMotor.getKrakenX60Foc(1);
    private final DCMotor feederMotorModel = DCMotor.getKrakenX60Foc(1);

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

    private ProfiledPIDController hoodFeedback;
    private PIDController flywheelFeedback;
    private PIDController feederFeedback;

    private SimpleMotorFeedforward flywheelFeedforward;
    private SimpleMotorFeedforward feederFeedforward;

    private boolean isHoodClosedLoop = true;
    private boolean isFlywheelClosedLoop = true;
    private boolean isFeederClosedLoop = true;

    private double desiredFlywheelVelocityRotationsPerSec = 0;
    private double desiredFeederVelocityRotationsPerSec = 0;

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

        // Initialize feedforward controllers with config values
        flywheelFeedforward = new SimpleMotorFeedforward(config.getFlywheelKS(), config.getFlywheelKV(), config.getFlywheelKA());
        feederFeedforward = new SimpleMotorFeedforward(config.getFeederKS(), config.getFeederKV(), config.getFeederKA());

        hoodFeedback.enableContinuousInput(-Math.PI, Math.PI);
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

        hoodSim.update(dt);
        flywheelSim.update(dt);
        feederSim.update(dt);

        inputs.hoodAngleRotations = hoodSim.getAngularPositionRotations();
        inputs.hoodVelocityRotationsPerSec = hoodSim.getAngularVelocityRadPerSec();

        inputs.flywheelVelocityRotationsPerSec = flywheelSim.getAngularVelocityRadPerSec() * 0.0485614385;
        inputs.flywheelAppliedVolts = flywheelSim.getInputVoltage();

        inputs.feederVelocityRotationsPerSec = feederSim.getAngularVelocityRadPerSec() * 0.0485614385;
        inputs.feederAppliedVolts = feederSim.getInputVoltage();

        inputs.hoodTorqueCurrent = hoodSim.getCurrentDrawAmps();

        // Simulation doesn't have temperature sensors, use default values
        inputs.hoodTemperatureFahrenheit = 70.0;
        inputs.flywheelTemperatureFahrenheit = 70.0;
        inputs.feederTemperatureFahrenheit = 70.0;
    }

    @Override
    public void setAngle(double angleRotations) {
        hoodFeedback.setGoal(angleRotations * (2 * Math.PI));
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
}

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.constants.shooter.ShooterConfigBase;
import frc.robot.lib.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX hoodMotor;
    private final TalonFX flywheelMotor;
    private final TalonFX feederMotor;

    private final StatusSignal<Angle> hoodPositionStatusSignal;
    private final StatusSignal<AngularVelocity> hoodVelocityStatusSignal;

    private final StatusSignal<AngularVelocity> flywheelVelocityStatusSignal;
    private final StatusSignal<AngularVelocity> feederVelocityStatusSignal;

    private final StatusSignal<Current> hoodTorqueCurrent;
    private final StatusSignal<Temperature> hoodTemperature;

    private final StatusSignal<Current> flywheelTorqueCurrent;
    private final StatusSignal<Temperature> flywheelTemperature;

    private final StatusSignal<Current> feederTorqueCurrent;
    private final StatusSignal<Temperature> feederTemperature;

    private final MotionMagicTorqueCurrentFOC hoodMotorRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
    private final VelocityTorqueCurrentFOC flywheelMotorRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
    private final VelocityTorqueCurrentFOC feederMotorRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);

    private final ShooterConfigBase config;

    public ShooterIOTalonFX(ShooterConfigBase config) {
        this.config = config;

        // Hood motor configuration (positional control)
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

        hoodConfig.Slot0.kP = config.getHoodKP();
        hoodConfig.Slot0.kI = config.getHoodKI();
        hoodConfig.Slot0.kD = config.getHoodKD();
        hoodConfig.Slot0.kS = config.getHoodKS();
        hoodConfig.Slot0.kV = config.getHoodKV();
        hoodConfig.Slot0.kA = config.getHoodKA();
        hoodConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = config.getHoodMotionMagicCruiseVelocityRotationsPerSec();
        hoodConfig.MotionMagic.MotionMagicAcceleration = config.getHoodMotionMagicAccelerationRotationsPerSecSec();
        hoodConfig.MotionMagic.MotionMagicJerk = config.getHoodMotionMagicJerkRotationsPerSecSecSec();

        hoodConfig.ClosedLoopGeneral.ContinuousWrap = true;
        hoodConfig.Feedback.SensorToMechanismRatio = config.getHoodMotorToOutputShaftRatio();

        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.MotorOutput.Inverted = config.getIsHoodInverted() ?
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        // Current and torque limiting
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = config.getHoodSupplyCurrentLimit();
        hoodConfig.CurrentLimits.SupplyCurrentLowerLimit = config.getHoodSupplyCurrentLimitLowerLimit();
        hoodConfig.CurrentLimits.SupplyCurrentLowerTime = config.getHoodSupplyCurrentLimitLowerTime();

        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.StatorCurrentLimit = config.getHoodStatorCurrentLimit();

        hoodConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getHoodPeakForwardTorqueCurrent();
        hoodConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.getHoodPeakReverseTorqueCurrent();

        // Software limits for hood angle
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.getHoodMaxAngleRotations();
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.getHoodMinAngleRotations();

        hoodConfig.FutureProofConfigs = true;

        hoodMotor = new TalonFX(config.getHoodCanId(), config.getCanBusName());
        PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig, 0.25));

        // Set starting angle offset
        PhoenixUtil.tryUntilOk(5, () -> hoodMotor.setPosition(config.getHoodStartingAngleRotations(), 0.25));

        // Flywheel motor configuration (velocity control)
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

        flywheelConfig.Slot0.kP = config.getFlywheelKP();
        flywheelConfig.Slot0.kI = config.getFlywheelKI();
        flywheelConfig.Slot0.kD = config.getFlywheelKD();
        flywheelConfig.Slot0.kS = config.getFlywheelKS();
        flywheelConfig.Slot0.kV = config.getFlywheelKV();
        flywheelConfig.Slot0.kA = config.getFlywheelKA();
        flywheelConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        flywheelConfig.ClosedLoopGeneral.ContinuousWrap = false;
        flywheelConfig.Feedback.SensorToMechanismRatio = config.getFlywheelMotorToOutputShaftRatio();

        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.MotorOutput.Inverted = config.getIsFlywheelInverted() ?
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        // Current and torque limiting
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = config.getFlywheelSupplyCurrentLimit();
        flywheelConfig.CurrentLimits.SupplyCurrentLowerLimit = config.getFlywheelSupplyCurrentLimitLowerLimit();
        flywheelConfig.CurrentLimits.SupplyCurrentLowerTime = config.getFlywheelSupplyCurrentLimitLowerTime();

        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = config.getFlywheelStatorCurrentLimit();

        flywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getFlywheelPeakForwardTorqueCurrent();
        flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.getFlywheelPeakReverseTorqueCurrent();

        flywheelConfig.FutureProofConfigs = true;

        flywheelMotor = new TalonFX(config.getFlywheelCanId(), config.getCanBusName());
        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig, 0.25));

        // Feeder motor configuration (velocity control)
        TalonFXConfiguration feederConfig = new TalonFXConfiguration();

        feederConfig.Slot0.kP = config.getFeederKP();
        feederConfig.Slot0.kI = config.getFeederKI();
        feederConfig.Slot0.kD = config.getFeederKD();
        feederConfig.Slot0.kS = config.getFeederKS();
        feederConfig.Slot0.kV = config.getFeederKV();
        feederConfig.Slot0.kA = config.getFeederKA();
        feederConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        feederConfig.ClosedLoopGeneral.ContinuousWrap = false;
        feederConfig.Feedback.SensorToMechanismRatio = config.getFeederMotorToOutputShaftRatio();

        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feederConfig.MotorOutput.Inverted = config.getIsFeederInverted() ?
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        // Current and torque limiting
        feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        feederConfig.CurrentLimits.SupplyCurrentLimit = config.getFeederSupplyCurrentLimit();
        feederConfig.CurrentLimits.SupplyCurrentLowerLimit = config.getFeederSupplyCurrentLimitLowerLimit();
        feederConfig.CurrentLimits.SupplyCurrentLowerTime = config.getFeederSupplyCurrentLimitLowerTime();

        feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        feederConfig.CurrentLimits.StatorCurrentLimit = config.getFeederStatorCurrentLimit();

        feederConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getFeederPeakForwardTorqueCurrent();
        feederConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.getFeederPeakReverseTorqueCurrent();

        feederConfig.FutureProofConfigs = true;

        feederMotor = new TalonFX(config.getFeederCanId(), config.getCanBusName());
        PhoenixUtil.tryUntilOk(5, () -> feederMotor.getConfigurator().apply(feederConfig, 0.25));

        // Status signals
        hoodTorqueCurrent = hoodMotor.getTorqueCurrent().clone();
        hoodTemperature = hoodMotor.getDeviceTemp().clone();

        flywheelTorqueCurrent = flywheelMotor.getTorqueCurrent().clone();
        flywheelTemperature = flywheelMotor.getDeviceTemp().clone();

        feederTorqueCurrent = feederMotor.getTorqueCurrent().clone();
        feederTemperature = feederMotor.getDeviceTemp().clone();

        hoodPositionStatusSignal = hoodMotor.getPosition().clone();
        hoodVelocityStatusSignal = hoodMotor.getVelocity().clone();

        flywheelVelocityStatusSignal = flywheelMotor.getVelocity().clone();
        feederVelocityStatusSignal = feederMotor.getVelocity().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(100,
            hoodTorqueCurrent, hoodTemperature,
            flywheelTorqueCurrent, flywheelTemperature,
            feederTorqueCurrent, feederTemperature,
            hoodPositionStatusSignal, hoodVelocityStatusSignal,
            flywheelVelocityStatusSignal, feederVelocityStatusSignal);

        hoodMotor.optimizeBusUtilization();
        flywheelMotor.optimizeBusUtilization();
        feederMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            hoodTorqueCurrent, hoodTemperature,
            flywheelTorqueCurrent, flywheelTemperature,
            feederTorqueCurrent, feederTemperature,
            hoodPositionStatusSignal, hoodVelocityStatusSignal,
            flywheelVelocityStatusSignal, feederVelocityStatusSignal);

        inputs.hoodAngleRotations = hoodPositionStatusSignal.getValue().in(Rotations);
        inputs.hoodVelocityRotationsPerSec = hoodVelocityStatusSignal.getValue().in(RotationsPerSecond);

        inputs.flywheelVelocityRotationsPerSec = flywheelVelocityStatusSignal.getValue().in(RotationsPerSecond);
        inputs.flywheelAppliedVolts = flywheelMotor.getMotorVoltage().getValueAsDouble();

        inputs.feederVelocityRotationsPerSec = feederVelocityStatusSignal.getValue().in(RotationsPerSecond);
        inputs.feederAppliedVolts = feederMotor.getMotorVoltage().getValueAsDouble();

        inputs.hoodTorqueCurrent = hoodTorqueCurrent.getValue().in(Amps);
        inputs.hoodTemperatureFahrenheit = hoodTemperature.getValue().in(Fahrenheit);

        inputs.flywheelTemperatureFahrenheit = flywheelTemperature.getValue().in(Fahrenheit);

        inputs.feederTemperatureFahrenheit = feederTemperature.getValue().in(Fahrenheit);
    }

    @Override
    public void setAngle(double angleRotations) {
        // Clamp angle within software limits
        double clampedAngle = Math.max(config.getHoodMinAngleRotations(),
            Math.min(config.getHoodMaxAngleRotations(), angleRotations));
        hoodMotor.setControl(hoodMotorRequest.withPosition(clampedAngle));
    }

    @Override
    public void setShotVelocity(double velocityRotationsPerSec) {
        flywheelMotor.setControl(flywheelMotorRequest.withVelocity(velocityRotationsPerSec));
    }

    @Override
    public void setFeedVelocity(double velocityRotationsPerSec) {
        feederMotor.setControl(feederMotorRequest.withVelocity(velocityRotationsPerSec));
    }

    @Override
    public void setHoodTorqueCurrentFOC(double torqueCurrentFOC) {
        hoodMotor.setControl(new TorqueCurrentFOC(torqueCurrentFOC));
    }

    @Override
    public void setFlywheelVoltage(double voltage) {
        flywheelMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setFeederVoltage(double voltage) {
        feederMotor.setControl(new VoltageOut(voltage));
    }
}

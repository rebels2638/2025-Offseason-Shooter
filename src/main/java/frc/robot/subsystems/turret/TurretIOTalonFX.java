package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Radians;
import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.turret.TurretConfigBase;
import frc.robot.subsystems.swerve.PhoenixOdometryThread;
import frc.robot.lib.util.PhoenixUtil;
import frc.robot.subsystems.swerve.SwerveDrive;

public class TurretIOTalonFX implements TurretIO {
    private final TalonFX turretMotor;

    private final Queue<Double> timestampQueue;

    private final StatusSignal<Angle> turretPositionStatusSignal;
    private final StatusSignal<AngularVelocity> turretVelocityStatusSignal;

    private final StatusSignal<Current> turretTorqueCurrent;
    private final StatusSignal<Temperature> turretTemperature;

    private final Queue<Double>  turretPositionQueue;

    private final MotionMagicExpoTorqueCurrentFOC turretMotorRequest = new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);
    private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0);

    private final TurretConfigBase generalConfig;

    private Rotation2d lastturretAngleRad = new Rotation2d();
    private SwerveModuleState lastRequestedState = new SwerveModuleState();
    private double lastRequestedStateTime = Timer.getFPGATimestamp();

    public TurretIOTalonFX(TurretConfigBase generalConfig) {
        this.generalConfig = generalConfig;

        // turret motor
        TalonFXConfiguration turretConfig = new TalonFXConfiguration();

        // Motion magic expo
        turretConfig.Slot0.kP = generalConfig.getTurretKP();
        turretConfig.Slot0.kI = generalConfig.getTurretKI();
        turretConfig.Slot0.kD = generalConfig.getTurretKD();
        turretConfig.Slot0.kS = generalConfig.getTurretKS();
        turretConfig.Slot0.kV = generalConfig.getTurretKV();
        turretConfig.Slot0.kA = generalConfig.getTurretKA();
        turretConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        turretConfig.MotionMagic.MotionMagicExpo_kA = generalConfig.getTurretMotionMagicExpoKA();
        turretConfig.MotionMagic.MotionMagicExpo_kV = generalConfig.getTurretMotionMagicExpoKV();
        turretConfig.MotionMagic.MotionMagicCruiseVelocity = generalConfig.getTurretMotionMagicCruiseVelocityRotationsPerSec();

        turretConfig.MotorOutput.NeutralMode = 
            generalConfig.getIsTurretNeutralModeBrake() ? 
                NeutralModeValue.Brake : 
                NeutralModeValue.Coast;

        turretConfig.MotorOutput.Inverted = 
            generalConfig.getIsTurretInverted() ?
                InvertedValue.Clockwise_Positive :
                InvertedValue.CounterClockwise_Positive;
                
        // Cancoder + encoder
        turretConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turretConfig.Feedback.FeedbackRemoteSensorID = generalConfig.getCancoderCanId();
        turretConfig.Feedback.FeedbackSensorSource = generalConfig.getTurretCancoderFeedbackSensorSource();
        turretConfig.Feedback.SensorToMechanismRatio = 1;
        turretConfig.Feedback.RotorToSensorRatio = generalConfig.getTurretRotorToSensorRatio();

        // current and torque limiting
        turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turretConfig.CurrentLimits.SupplyCurrentLimit = generalConfig.getTurretSupplyCurrentLimit();
        turretConfig.CurrentLimits.SupplyCurrentLowerLimit = generalConfig.getTurretSupplyCurrentLimitLowerLimit();
        turretConfig.CurrentLimits.SupplyCurrentLowerTime = generalConfig.getTurretSupplyCurrentLimitLowerTime();

        turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turretConfig.CurrentLimits.StatorCurrentLimit = generalConfig.getTurretStatorCurrentLimit();

        turretConfig.TorqueCurrent.PeakForwardTorqueCurrent = generalConfig.getTurretPeakForwardTorqueCurrent();
        turretConfig.TorqueCurrent.PeakReverseTorqueCurrent = generalConfig.getTurretPeakReverseTorqueCurrent();

        turretConfig.FutureProofConfigs = true;

        turretMotor = new TalonFX(generalConfig.getTurretCanId(), generalConfig.getCanBusName());
        PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(turretConfig, 0.25));
        
        turretTorqueCurrent = turretMotor.getTorqueCurrent().clone();
        turretTemperature = turretMotor.getDeviceTemp().clone();

        turretPositionStatusSignal = turretMotor.getPosition().clone();
        turretVelocityStatusSignal = turretMotor.getVelocity().clone();

        turretPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turretPositionStatusSignal.clone());

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            turretTorqueCurrent,
            turretTemperature
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            SwerveDrive.ODOMETRY_FREQUENCY,

            turretPositionStatusSignal,
            turretVelocityStatusSignal
        );

        turretMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            turretTorqueCurrent,
            turretTemperature,

            turretPositionStatusSignal,
            turretVelocityStatusSignal
        );

        inputs.turretPosition = new Rotation2d(BaseStatusSignal.getLatencyCompensatedValue(turretPositionStatusSignal, turretVelocityStatusSignal).in(Radians));
        inputs.turretVelocityRadPerSec = turretVelocityStatusSignal.getValue().in(RadiansPerSecond);

        inputs.turretTorqueCurrent = turretTorqueCurrent.getValue().in(Amps);
        inputs.turretTemperatureFahrenheit = turretTemperature.getValue().in(Fahrenheit);

        inputs.odometryTimestampsSeconds = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.odometryTurretPositions = turretPositionQueue.stream().map((Double value) -> Rotation2d.fromRotations(value)).toArray(Rotation2d[]::new);

        timestampQueue.clear();
        turretPositionQueue.clear();

        lastturretAngleRad = new Rotation2d(inputs.turretPosition.getRadians());
    }

    @Override
    public void setState(SwerveModuleState state) {
        turretMotor.setControl(
            turretMotorRequest.withPosition(
                state.angle.getRotations()
            )
        );

        lastRequestedState = state;
        lastRequestedStateTime = Timer.getFPGATimestamp();
    }

    @Override
    public void setTurretTorqueCurrentFOC(double torqueCurrentFOC) {
        // Set turret motor with torque FOC, optionally using drive velocity for feedforward if needed
        turretMotor.setControl(
            torqueCurrentFOCRequest.withOutput(torqueCurrentFOC)
        );
    }
}
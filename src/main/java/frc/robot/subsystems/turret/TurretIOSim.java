package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.turret.TurretConfigBase;

public class TurretIOSim implements TurretIO {
    private final DCMotor turnMotorModel = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim turretSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnMotorModel,  0.00015, 21.428), // magic number because turret is not important
            turnMotorModel
        );

    private PIDController turretFeedback = new PIDController(25, 0.0, 0.0);

    private boolean isTurretClosedLoop = true;

    private SwerveModuleState lastDesiredState = new SwerveModuleState();

    private double lastTimeInputs = Timer.getTimestamp();
    
    public TurretIOSim(TurretConfigBase config) {
        turretFeedback.setTolerance(Units.degreesToRadians(2));
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        double dt = Timer.getTimestamp() - lastTimeInputs;
        lastTimeInputs = Timer.getTimestamp();

        if (isTurretClosedLoop) {
            turretSim.setInputVoltage(
                MathUtil.clamp(
                    turretFeedback.calculate(turretSim.getAngularPositionRad()),
                    -12,
                    12
                )
            );
        }

        turretSim.update(dt);

        inputs.turretPosition = new Rotation2d(turretSim.getAngularPosition());
        inputs.turretVelocityRadPerSec = turretSim.getAngularVelocityRadPerSec();

        inputs.turretTorqueCurrent = turretSim.getCurrentDrawAmps();

        inputs.odometryTimestampsSeconds = new double[] {Timer.getTimestamp()};
        inputs.odometryTurretPositions = new Rotation2d[] {inputs.turretPosition};
    }

    @Override
    public void setState(SwerveModuleState state) {
        turretFeedback.setSetpoint(state.angle.getRadians());

        isTurretClosedLoop = false;

        lastDesiredState = state;
    }

    @Override
    public void setTurretTorqueCurrentFOC(double torqueCurrentFOC) {
        // In sim, treat torqueCurrentFOC as voltage for simplicity
        turretSim.setInputVoltage(torqueCurrentFOC);

        isTurretClosedLoop = true;
    }
}
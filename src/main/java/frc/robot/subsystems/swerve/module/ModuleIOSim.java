package frc.robot.subsystems.swerve.module;

import java.io.Console;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.swerve.moduleConfigs.SwerveModuleGeneralConfigBase;
import frc.robot.lib.util.RebelUtil;

public class ModuleIOSim implements ModuleIO {
    private final DCMotor driveMotorModel = DCMotor.getKrakenX60Foc(1);
    private final DCMotor turnMotorModel = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim driveSim =
        new DCMotorSim( 
            LinearSystemId.createDCMotorSystem(driveMotorModel, (2.8087 * 0.0194 * 0.0485614385) / 6.12, 6.12), // J = (kA_linear * Kt * r) / G
            driveMotorModel
        );
    private final DCMotorSim steerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnMotorModel,  0.00015, 21.428), // magic number because steer is not important
            turnMotorModel
        );


    private PIDController driveFeedback = new PIDController(0.1, 0.0, 0.01);
    private PIDController steerFeedback = new PIDController(0.2, 0.0, 0.02);

    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.5, 1.2, 0.1);
    private SimpleMotorFeedforward steerFeedforward = new SimpleMotorFeedforward(0.3, 0.8, 0.05);

    private boolean isSteerClosedLoop = true;
    private boolean isDriveClosedLoop = true;

    private SwerveModuleState lastDesiredState = new SwerveModuleState();

    private double lastTimeInputs = Timer.getTimestamp();
    
    private final int moduleID;
    public ModuleIOSim(SwerveModuleGeneralConfigBase config, int moduleID) {
        this.moduleID = moduleID;
        steerFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        double dt = Timer.getTimestamp() - lastTimeInputs;
        lastTimeInputs = Timer.getTimestamp();

        if (isDriveClosedLoop) {
            driveSim.setInputVoltage(
                MathUtil.clamp(
                    driveFeedforward.calculate(lastDesiredState.speedMetersPerSecond) +
                    driveFeedback.calculate(
                        driveSim.getAngularVelocityRadPerSec() * 0.0485614385 // wheel radius in meters, 
                        lastDesiredState.speedMetersPerSecond
                    ),
                    -12,
                    12
                )
            );
        }


        steerSim.update(dt);
        driveSim.update(dt);

        inputs.fpgaTimestampSeconds = Timer.getTimestamp();

        

    }

    @Override
    public void setState(SwerveModuleState state) {
        driveFeedback.setSetpoint(state.speedMetersPerSecond);
        steerFeedback.setSetpoint(state.angle.getRadians());

        isDriveClosedLoop = true;
        isSteerClosedLoop = true;

        lastDesiredState = state;
    }

    @Override
    public void setSteerTorqueCurrentFOC(double torqueCurrentFOC, double driveVelocityMetersPerSec) {
        // In sim, treat torqueCurrentFOC as voltage for simplicity
        steerSim.setInputVoltage(torqueCurrentFOC);

        isDriveClosedLoop = false;
        isSteerClosedLoop = true;
    }

    @Override
    public void setDriveTorqueCurrentFOC(double torqueCurrentFOC, Rotation2d steerAngle) {
        // In sim, treat torqueCurrentFOC as voltage for simplicity
        driveSim.setInputVoltage(torqueCurrentFOC);

        isDriveClosedLoop = false;
        isSteerClosedLoop = true;
    }
}

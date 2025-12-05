package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotState;
import frc.robot.RobotState.OdometryObservation;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigBase;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigComp;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigProto;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigSim;
import frc.robot.constants.swerve.moduleConfigs.SwerveModuleGeneralConfigBase;
import frc.robot.constants.swerve.moduleConfigs.comp.SwerveModuleGeneralConfigComp;
import frc.robot.constants.swerve.moduleConfigs.comp.SwerveModuleSpecificBLConfigComp;
import frc.robot.constants.swerve.moduleConfigs.comp.SwerveModuleSpecificBRConfigComp;
import frc.robot.constants.swerve.moduleConfigs.comp.SwerveModuleSpecificFLConfigComp;
import frc.robot.constants.swerve.moduleConfigs.comp.SwerveModuleSpecificFRConfigComp;
import frc.robot.constants.swerve.moduleConfigs.proto.SwerveModuleGeneralConfigProto;
import frc.robot.constants.swerve.moduleConfigs.proto.SwerveModuleSpecificBLConfigProto;
import frc.robot.constants.swerve.moduleConfigs.proto.SwerveModuleSpecificFRConfigProto;
import frc.robot.constants.swerve.moduleConfigs.sim.SwerveModuleGeneralConfigSim;
import frc.robot.lib.BLine.ChassisRateLimiter;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOInputsAutoLogged;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.module.ModuleIOTalonFX;

public class SwerveDrive extends SubsystemBase {
    private static SwerveDrive instance = null;
    public static SwerveDrive getInstance() {
        if (instance == null) {
            instance = new SwerveDrive();
        }

        return instance;
    }

    // FSM State Enums
    public enum DesiredState {
        STOPPED,
        TELEOP,
        RANGED_ROTATION_TELEOP,
        FOLLOW_PATH,
        PREPARE_FOR_AUTO,
        RANGED_ROTATION_FOLLOW_PATH,
        SYSID
    }

    public enum CurrentState {
        STOPPED,
        TELEOP,
        NOMINAL_RANGED_ROTATION_TELEOP,
        RETURNING_RANGED_ROTATION_TELEOP,
        FOLLOW_PATH,
        PREPARE_FOR_AUTO,
        NOMINAL_RANGED_ROTATION_FOLLOW_PATH,
        RETURNING_RANGED_ROTATION_FOLLOW_PATH,
        SYSID
    }

    // FSM State Variables
    private DesiredState desiredState = DesiredState.STOPPED;
    private CurrentState currentState = CurrentState.STOPPED;

    // Teleop input suppliers (normalized -1 to 1)
    private DoubleSupplier vxNormalizedSupplier = () -> 0.0;
    private DoubleSupplier vyNormalizedSupplier = () -> 0.0;
    private DoubleSupplier omegaNormalizedSupplier = () -> 0.0;

    // Path following
    private Supplier<Path> pathSupplier = () -> null;
    private Supplier<Boolean> shouldPoseResetSupplier = () -> false;
    private Command currentPathCommand = null;
    private FollowPath.Builder followPathBuilder;

    // Ranged rotation
    private Rotation2d rotationRangeMin = Rotation2d.fromDegrees(-180);
    private Rotation2d rotationRangeMax = Rotation2d.fromDegrees(180);
    private PIDController rangedRotationPIDController;
    private static final double RANGED_ROTATION_MAX_VELOCITY_FACTOR = 0.6;
    private static final double RANGED_ROTATION_BUFFER_RAD = Math.toRadians(5.0); // Buffer to prevent oscillation at boundaries

    // Omega override for drive shim (used for ranged rotation during path following)
    private Double omegaOverride = null;

    // Alliance-based inversion
    private int invert = 1;

    public static final double ODOMETRY_FREQUENCY = 250;

    public static final Lock odometryLock = new ReentrantLock();

    private ModuleIO[] modules;
    private ModuleIOInputsAutoLogged[] moduleInputs = {
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged()
    };

    private final GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };  
    private SwerveModuleState[] moduleStates = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private ChassisSpeeds desiredRobotRelativeSpeeds = new ChassisSpeeds();
    private ChassisSpeeds obtainableFieldRelativeSpeeds = new ChassisSpeeds();

    double prevLoopTime = Timer.getTimestamp();
    double prevDriveTime = Timer.getTimestamp();

    private final SwerveModuleGeneralConfigBase moduleGeneralConfig;
    private final SwerveDrivetrainConfigBase drivetrainConfig;
    private SwerveDriveKinematics kinematics;

    private final SysIdRoutine driveCharacterizationSysIdRoutine;
    private final SysIdRoutine steerCharacterizationSysIdRoutine;

    @SuppressWarnings("static-access")
    private SwerveDrive() {
        switch (Constants.currentMode) {
            case COMP:
                drivetrainConfig = SwerveDrivetrainConfigComp.getInstance();
                moduleGeneralConfig = SwerveModuleGeneralConfigComp.getInstance();

                modules = new ModuleIO[] {
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificFLConfigComp.getInstance()),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificFRConfigComp.getInstance()),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificBLConfigComp.getInstance()),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificBRConfigComp.getInstance())
                };
                
                gyroIO = new GyroIOPigeon2();
                PhoenixOdometryThread.getInstance().start();

                break;

            case PROTO:
                drivetrainConfig = SwerveDrivetrainConfigProto.getInstance();
                moduleGeneralConfig = SwerveModuleGeneralConfigProto.getInstance();

                modules = new ModuleIO[] {
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificBLConfigProto.getInstance()),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificFRConfigProto.getInstance()),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificBLConfigProto.getInstance()),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificBLConfigProto.getInstance())
                };
                
                gyroIO = new GyroIOPigeon2();
                PhoenixOdometryThread.getInstance().start();
                break;

            case SIM:
                drivetrainConfig = SwerveDrivetrainConfigSim.getInstance();
                moduleGeneralConfig = SwerveModuleGeneralConfigSim.getInstance();

                modules = new ModuleIO[] {
                    new ModuleIOSim(moduleGeneralConfig, 0),
                    new ModuleIOSim(moduleGeneralConfig, 1),
                    new ModuleIOSim(moduleGeneralConfig, 2),
                    new ModuleIOSim(moduleGeneralConfig, 3)
                };

                gyroIO = new GyroIO() {};
                break;

            case REPLAY:
                drivetrainConfig = SwerveDrivetrainConfigComp.getInstance();
                moduleGeneralConfig = SwerveModuleGeneralConfigComp.getInstance();

                modules = new ModuleIO[] {
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {}
                };

                gyroIO = new GyroIOPigeon2();

                break;

            default:
                drivetrainConfig = SwerveDrivetrainConfigComp.getInstance();
                moduleGeneralConfig = SwerveModuleGeneralConfigComp.getInstance();

                modules = new ModuleIO[] {
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificFLConfigComp.getInstance()),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificFRConfigComp.getInstance()),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificBLConfigComp.getInstance()),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificBRConfigComp.getInstance())
                };
                
                gyroIO = new GyroIOPigeon2();
                PhoenixOdometryThread.getInstance().start();

                break;
                
        }

        kinematics = new SwerveDriveKinematics(
            drivetrainConfig.getFrontLeftPositionMeters(),
            drivetrainConfig.getFrontRightPositionMeters(),
            drivetrainConfig.getBackLeftPositionMeters(),
            drivetrainConfig.getBackRightPositionMeters()
        );

        // Create the SysId routine - this is going to be in torque current foc units not voltage
        driveCharacterizationSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1.5).per(Second), Volts.of(12), Seconds.of(15), // Use default config
                (state) -> Logger.recordOutput("DriveCharacterizationSysIdRoutineState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (torqueCurrentFOC) -> {
                    for (ModuleIO module : modules) {
                        module.setDriveTorqueCurrentFOC(torqueCurrentFOC.in(Volts), new Rotation2d(0));
                    }
                },
                null, // No log consumer, since data is recorded by AdvantageKit
                this
            )
        );

        steerCharacterizationSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second), Volts.of(7), Seconds.of(10), // Use default config
                (state) -> Logger.recordOutput("SteerCharacterizationSysIdRoutineState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (torqueCurrentFOC) -> {
                    for (ModuleIO module : modules) {
                        module.setSteerTorqueCurrentFOC(torqueCurrentFOC.in(Volts), 0);
                    }
                },
                null, // No log consumer, since data is recorded by AdvantageKit
                this
            )
        );

        // Configure FollowPath builder using drivetrain config
        followPathBuilder = new FollowPath.Builder(
            this,
            RobotState.getInstance()::getEstimatedPose,
            RobotState.getInstance()::getRobotRelativeSpeeds,
            this::driveRobotRelativeShim,
            new PIDController(
                drivetrainConfig.getFollowPathTranslationKP(),
                drivetrainConfig.getFollowPathTranslationKI(),
                drivetrainConfig.getFollowPathTranslationKD()
            ),
            new PIDController(
                drivetrainConfig.getFollowPathRotationKP(),
                drivetrainConfig.getFollowPathRotationKI(),
                drivetrainConfig.getFollowPathRotationKD()
            ),
            new PIDController(
                drivetrainConfig.getFollowPathCrossTrackKP(),
                drivetrainConfig.getFollowPathCrossTrackKI(),
                drivetrainConfig.getFollowPathCrossTrackKD()
            )
        ).withDefaultShouldFlip();

        // Configure ranged rotation PID controller with velocity limiting
        rangedRotationPIDController = new PIDController(
            drivetrainConfig.getRangedRotationKP(),
            drivetrainConfig.getRangedRotationKI(),
            drivetrainConfig.getRangedRotationKD()
        );
        rangedRotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
        rangedRotationPIDController.setTolerance(Math.toRadians(drivetrainConfig.getRangedRotationToleranceDeg()));

    }

    @Override
    public void periodic() {
        double dt = Timer.getTimestamp() - prevLoopTime; 
        prevLoopTime = Timer.getTimestamp();

        Logger.recordOutput("SwerveDrive/dtPeriodic", dt);

        // Thread safe reading of the gyro and swerve inputs.
        // The read lock is released only after inputs are written via the write lock
        odometryLock.lock();
        try {
            Logger.recordOutput("SwerveDrive/stateLockAcquired", true);
            gyroIO.updateInputs(gyroInputs);
            Logger.processInputs("SwerveDrive/gyro", gyroInputs);

            for (int i = 0; i < 4; i++) {
                modules[i].updateInputs(moduleInputs[i]);
                Logger.processInputs("SwerveDrive/module" + i, moduleInputs[i]);
            }
        } finally {
            odometryLock.unlock();
        }

        for (int i = 0; i < 4; i++) {
            moduleStates[i] = new SwerveModuleState(
                moduleInputs[i].driveVelocityMetersPerSec,
                moduleInputs[i].steerPosition
            );
        }

        ArrayList<Pose2d> updatedPoses = new ArrayList<Pose2d>();

        double[] odometryTimestampsSeconds = moduleInputs[0].odometryTimestampsSeconds;
        for (int i = 0; i < odometryTimestampsSeconds.length; i++) {
            for (int j = 0; j < 4; j++) {
                modulePositions[j] = new SwerveModulePosition(
                    moduleInputs[j].odometryDrivePositionsMeters[i],
                    moduleInputs[j].odometrySteerPositions[i]
                );
            }
            
            RobotState.getInstance().addOdometryObservation(
                new OdometryObservation(
                    odometryTimestampsSeconds[i],
                    gyroInputs.isConnected,
                    modulePositions,
                    moduleStates,
                    gyroInputs.isConnected ? gyroInputs.odometryYawPositions[i] : new Rotation2d(),
                    gyroInputs.isConnected ? gyroInputs.yawVelocityRadPerSec : 0
                )
            );

            updatedPoses.add(RobotState.getInstance().getEstimatedPose());
        }

        Logger.recordOutput("SwerveDrive/updatedPoses", updatedPoses.toArray(new Pose2d[0]));
        Logger.recordOutput("SwerveDrive/measuredModuleStates", moduleStates);
        Logger.recordOutput("SwerveDrive/measuredModulePositions", modulePositions);

        // FSM processing
        handleStateTransitions();
        handleCurrentState();

        Logger.recordOutput("SwerveDrive/CurrentCommand", this.getCurrentCommand() == null ? "" : this.getCurrentCommand().toString());
    }

    /**
     * Determines the next measured state based on the desired state.
     */
    private void handleStateTransitions() {
        CurrentState previousState = currentState;

        switch (desiredState) {
            case STOPPED:
                currentState = CurrentState.STOPPED;
                break;

            case TELEOP:
                currentState = CurrentState.TELEOP;
                break;

            case RANGED_ROTATION_TELEOP:
                if (isWithinRotationRange()) {
                    currentState = CurrentState.NOMINAL_RANGED_ROTATION_TELEOP;
                } else {
                    currentState = CurrentState.RETURNING_RANGED_ROTATION_TELEOP;
                }
                break;

            case FOLLOW_PATH:
                currentState = CurrentState.FOLLOW_PATH;
                break;

            case PREPARE_FOR_AUTO:
                currentState = CurrentState.PREPARE_FOR_AUTO;
                break;

            case RANGED_ROTATION_FOLLOW_PATH:
                if (isWithinRotationRange()) {
                    currentState = CurrentState.NOMINAL_RANGED_ROTATION_FOLLOW_PATH;
                } else {
                    currentState = CurrentState.RETURNING_RANGED_ROTATION_FOLLOW_PATH;
                }
                break;

            case SYSID:
                currentState = CurrentState.SYSID;
                break;
        }

        // Handle state transition actions
        if (previousState != currentState) {
            handleStateExit(previousState);
            handleStateEntry(currentState, previousState);
        }
    }

    /**
     * Handle setup when entering a state.
     * Only sets wheel coast when transitioning between STOPPED and enabled states.
     */
    private void handleStateEntry(CurrentState enteringState, CurrentState exitingState) {
        // Only change wheel coast when transitioning between STOPPED and enabled states
        boolean wasEnabled = exitingState != CurrentState.STOPPED;
        boolean isEnabled = enteringState != CurrentState.STOPPED;
        
        if (wasEnabled != isEnabled) {
            setWheelCoast(!isEnabled);
        }
    }

    /**
     * Handle cleanup when exiting a state.
     */
    private void handleStateExit(CurrentState exitingState) {
        switch (exitingState) {
            case FOLLOW_PATH:
            case NOMINAL_RANGED_ROTATION_FOLLOW_PATH:
            case RETURNING_RANGED_ROTATION_FOLLOW_PATH:
                // Cancel the path command if transitioning away from follow path states
                if (currentPathCommand != null && currentPathCommand.isScheduled()) {
                    currentPathCommand.cancel();
                    currentPathCommand = null;
                }
                // Clear omega override when leaving ranged rotation follow path states
                omegaOverride = null;
                break;
            default:
                break;
        }
    }

    /**
     * Executes behavior for the current state.
     */
    private void handleCurrentState() {
        switch (currentState) {
            case STOPPED:
                handleStoppedState();
                break;
            case TELEOP:
                handleTeleopState();
                break;
            case NOMINAL_RANGED_ROTATION_TELEOP:
                handleNominalRangedRotationTeleopState();
                break;
            case RETURNING_RANGED_ROTATION_TELEOP:
                handleReturningRangedRotationTeleopState();
                break;
            case FOLLOW_PATH:
                handleFollowPathState();
                break;
            case PREPARE_FOR_AUTO:
                handlePrepareForAutoState();
                break;
            case NOMINAL_RANGED_ROTATION_FOLLOW_PATH:
                handleNominalRangedRotationFollowPathState();
                break;
            case RETURNING_RANGED_ROTATION_FOLLOW_PATH:
                handleReturningRangedRotationFollowPathState();
                break;
            case SYSID:
                handleSysIdState();
                break;
        }
    }

    private void handleStoppedState() {
        driveFieldRelative(new ChassisSpeeds(0, 0, 0));
    }

    private void handleTeleopState() {
        // Update alliance inversion
        invert = Constants.shouldFlipPath() ? -1 : 1;
        
        // Calculate speeds from normalized inputs
        ChassisSpeeds desiredFieldRelativeSpeeds = new ChassisSpeeds(
            vxNormalizedSupplier.getAsDouble() * drivetrainConfig.getMaxTranslationalVelocityMetersPerSec() * invert,
            vyNormalizedSupplier.getAsDouble() * drivetrainConfig.getMaxTranslationalVelocityMetersPerSec() * invert,
            omegaNormalizedSupplier.getAsDouble() * drivetrainConfig.getMaxAngularVelocityRadiansPerSec()
        );
        Logger.recordOutput("SwerveDrive/TeleopDesiredFieldRelativeSpeeds", desiredFieldRelativeSpeeds);

        driveFieldRelative(desiredFieldRelativeSpeeds);
    }

    private void handleNominalRangedRotationTeleopState() {
        // Update alliance inversion
        invert = Constants.shouldFlipPath() ? -1 : 1;
        
        double desiredOmega = omegaNormalizedSupplier.getAsDouble() * drivetrainConfig.getMaxAngularVelocityRadiansPerSec();
        double limitedOmega = limitOmegaForRange(desiredOmega);
        
        ChassisSpeeds desiredFieldRelativeSpeeds = new ChassisSpeeds(
            vxNormalizedSupplier.getAsDouble() * drivetrainConfig.getMaxTranslationalVelocityMetersPerSec() * invert,
            vyNormalizedSupplier.getAsDouble() * drivetrainConfig.getMaxTranslationalVelocityMetersPerSec() * invert,
            limitedOmega
        );
        Logger.recordOutput("SwerveDrive/RangedTeleopDesiredFieldRelativeSpeeds", desiredFieldRelativeSpeeds);

        driveFieldRelative(desiredFieldRelativeSpeeds);
    }

    private void handleReturningRangedRotationTeleopState() {
        // Update alliance inversion
        invert = Constants.shouldFlipPath() ? -1 : 1;
        
        // Use profiled PID to return to range
        double correctionOmega = calculateReturnToRangeOmega();
        
        ChassisSpeeds desiredFieldRelativeSpeeds = new ChassisSpeeds(
            vxNormalizedSupplier.getAsDouble() * drivetrainConfig.getMaxTranslationalVelocityMetersPerSec() * invert,
            vyNormalizedSupplier.getAsDouble() * drivetrainConfig.getMaxTranslationalVelocityMetersPerSec() * invert,
            correctionOmega
        );
        Logger.recordOutput("SwerveDrive/ReturningRangedTeleopDesiredFieldRelativeSpeeds", desiredFieldRelativeSpeeds);

        driveFieldRelative(desiredFieldRelativeSpeeds);
    }

    private void handleFollowPathState() {
        // Schedule path command if not already running
        Path path = pathSupplier.get();
        if (path != null && (currentPathCommand == null || !currentPathCommand.isScheduled())) {
            currentPathCommand = buildPathCommand(path);
            currentPathCommand.schedule();
        }
        // The path command handles driving via the callback
    }

    private void handlePrepareForAutoState() {
        // No-op placeholder for now
    }

    private void handleNominalRangedRotationFollowPathState() {
        // Clear omega override - the shim will use path's omega but limit it
        omegaOverride = null;
        
        // Schedule path command if not already running
        Path path = pathSupplier.get();
        if (path != null && (currentPathCommand == null || !currentPathCommand.isScheduled())) {
            currentPathCommand = buildPathCommand(path);
            currentPathCommand.schedule();
        }
    }

    private void handleReturningRangedRotationFollowPathState() {
        // Override omega for the path following shim
        omegaOverride = calculateReturnToRangeOmega();
        
        // Schedule path command if not already running
        Path path = pathSupplier.get();
        if (path != null && (currentPathCommand == null || !currentPathCommand.isScheduled())) {
            currentPathCommand = buildPathCommand(path);
            currentPathCommand.schedule();
        }
    }

    /**
     * Builds a path command using the followPathBuilder, applying pose reset if configured.
     */
    private Command buildPathCommand(Path path) {
        if (shouldPoseResetSupplier.get()) {
            return followPathBuilder.withPoseReset(RobotState.getInstance()::resetPose).build(path);
        }
        return followPathBuilder.withPoseReset((Pose2d pose) -> {}).build(path);
    }

    private void handleSysIdState() {
        // SysId routines handle their own motor control
        // This state just prevents other states from interfering
    }

    /**
     * Checks if robot rotation is within the specified range.
     */
    private boolean isWithinRotationRange() {
        Rotation2d currentRotation = RobotState.getInstance().getEstimatedPose().getRotation();
        
        // Normalize angles to -PI to PI for comparison
        double current = MathUtil.angleModulus(currentRotation.getRadians());
        double min = MathUtil.angleModulus(rotationRangeMin.getRadians());
        double max = MathUtil.angleModulus(rotationRangeMax.getRadians());
        
        // Handle wrap-around case
        if (min <= max) {
            return current >= min && current <= max;
        } else {
            // Range wraps around (e.g., min=170deg, max=-170deg)
            return current >= min || current <= max;
        }
    }

    /**
     * Limits omega using sqrt(2*a*d) formula to prevent exceeding rotation bounds.
     */
    private double limitOmegaForRange(double desiredOmega) {
        Rotation2d currentRotation = RobotState.getInstance().getEstimatedPose().getRotation();
        
        double current = currentRotation.getRadians();
        double min = rotationRangeMin.getRadians();
        double max = rotationRangeMax.getRadians();
        
        // Calculate distance to bounds (using Rotation2d for proper wrapping)
        double distToMin = Math.abs(MathUtil.angleModulus(current - min));
        double distToMax = Math.abs(MathUtil.angleModulus(max - current));
        
        double maxAngularAccel = drivetrainConfig.getMaxAngularAccelerationRadiansPerSecSec();
        
        // sqrt(2*a*d) formula for max velocity to stop at boundary
        double maxOmegaToMin = Math.sqrt(2 * maxAngularAccel * distToMin);
        double maxOmegaToMax = Math.sqrt(2 * maxAngularAccel * distToMax);
        
        // Clamp omega based on direction
        if (desiredOmega > 0) {
            // Rotating towards max bound
            return Math.min(desiredOmega, maxOmegaToMax);
        } else {
            // Rotating towards min bound
            return Math.max(desiredOmega, -maxOmegaToMin);
        }
    }

    /**
     * Calculates omega to return to rotation range using PID with velocity limiting.
     * Uses an internal buffer to target slightly inside the range to prevent oscillation at boundaries.
     */
    private double calculateReturnToRangeOmega() {
        Rotation2d currentRotation = RobotState.getInstance().getEstimatedPose().getRotation();
        
        double current = currentRotation.getRadians();
        double min = rotationRangeMin.getRadians();
        double max = rotationRangeMax.getRadians();
        
        // Find closest bound
        double distToMin = Math.abs(MathUtil.angleModulus(current - min));
        double distToMax = Math.abs(MathUtil.angleModulus(current - max));
        
        double targetAngle;
        if (distToMin < distToMax) {
            // Target slightly inside the min boundary (add buffer)
            targetAngle = min + RANGED_ROTATION_BUFFER_RAD;
        } else {
            // Target slightly inside the max boundary (subtract buffer)
            targetAngle = max - RANGED_ROTATION_BUFFER_RAD;
        }
        
        // Calculate PID output
        double pidOutput = rangedRotationPIDController.calculate(current, targetAngle);
        
        // Apply velocity limit (0.6 of max omega)
        double maxOmega = drivetrainConfig.getMaxAngularVelocityRadiansPerSec() * RANGED_ROTATION_MAX_VELOCITY_FACTOR;
        return MathUtil.clamp(pidOutput, -maxOmega, maxOmega);
    }

    // Supplier setters
    public void setTeleopInputSuppliers(
        DoubleSupplier vxNormalized,
        DoubleSupplier vyNormalized,
        DoubleSupplier omegaNormalized
    ) {
        this.vxNormalizedSupplier = vxNormalized;
        this.vyNormalizedSupplier = vyNormalized;
        this.omegaNormalizedSupplier = omegaNormalized;
    }

    public void setPathSupplier(Supplier<Path> pathSupplier) {
        this.pathSupplier = pathSupplier;
        this.shouldPoseResetSupplier = () -> false;
    }

    public void setPathSupplier(Supplier<Path> pathSupplier, Supplier<Boolean> shouldPoseResetSupplier) {
        this.pathSupplier = pathSupplier;
        this.shouldPoseResetSupplier = shouldPoseResetSupplier;
    }

    public void setRotationRange(Rotation2d min, Rotation2d max) {
        this.rotationRangeMin = min;
        this.rotationRangeMax = max;

        Logger.recordOutput("SwerveDrive/rotationRangeMin", min);
        Logger.recordOutput("SwerveDrive/rotationRangeMax", max);
    }

    // State getters/setters
    @AutoLogOutput(key = "SwerveDrive/desiredState")
    public DesiredState getDesiredState() {
        return desiredState;
    }

    @AutoLogOutput(key = "SwerveDrive/currentState")
    public CurrentState getCurrentState() {
        return currentState;
    }

    public void setDesiredState(DesiredState desiredState) {
        this.desiredState = desiredState;
    }

    // Existing drive methods
    private ChassisSpeeds compensateRobotRelativeSpeeds(ChassisSpeeds speeds) {
        Rotation2d angularVelocity = new Rotation2d(speeds.omegaRadiansPerSecond * drivetrainConfig.getRotationCompensationCoefficient());
        if (angularVelocity.getRadians() != 0.0) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                ChassisSpeeds.fromRobotRelativeSpeeds( // why should this be split into two?
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond,
                    RobotState.getInstance().getEstimatedPose().getRotation().plus(angularVelocity)
                ),
                RobotState.getInstance().getEstimatedPose().getRotation()
            );
        }

        return speeds;
    }

    // return a supplier that is true if the modules are aligned within the tolerance
    public Supplier<Boolean> alignModules(Rotation2d targetRotation, double toleranceDeg) {
        for (int i = 0; i < 4; i++) {
            SwerveModuleState state = new SwerveModuleState(0, targetRotation);
            state.optimize(moduleStates[i].angle);
            modules[i].setState(state);
        }

        return () -> {
            for (int i = 0; i < 4; i++) {
                if (Math.abs(moduleStates[i].angle.minus(targetRotation).getDegrees()) > toleranceDeg && 
                    Math.abs(moduleStates[i].angle.plus(Rotation2d.fromDegrees(180)).minus(targetRotation).getDegrees()) > toleranceDeg) {
                    return false;
                }
            }
            return true;
        };
    }
    
    /**
     * Shim for driveRobotRelative that allows omega override for ranged rotation during path following.
     * Omega override is managed by state handlers, not cleared automatically.
     */
    private void driveRobotRelativeShim(ChassisSpeeds speeds) {
        if (omegaOverride != null) {
            speeds = new ChassisSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                omegaOverride
            );
        }
        driveRobotRelative(speeds);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        double dt = Timer.getTimestamp() - prevDriveTime; 
        prevDriveTime = Timer.getTimestamp();

        desiredRobotRelativeSpeeds = speeds;

        ChassisSpeeds desiredFieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(desiredRobotRelativeSpeeds, RobotState.getInstance().getEstimatedPose().getRotation());
        Logger.recordOutput("SwerveDrive/desiredFieldRelativeSpeeds", desiredFieldRelativeSpeeds);
        Logger.recordOutput("SwerveDrive/desiredRobotRelativeSpeeds", desiredRobotRelativeSpeeds);
        
        // Limit acceleration to prevent sudden changes in speed
        obtainableFieldRelativeSpeeds = ChassisRateLimiter.limit(
            desiredFieldRelativeSpeeds, 
            obtainableFieldRelativeSpeeds, 
            dt, 
            drivetrainConfig.getMaxTranslationalAccelerationMetersPerSecSec(), 
            drivetrainConfig.getMaxAngularAccelerationRadiansPerSecSec(),
            drivetrainConfig.getMaxTranslationalVelocityMetersPerSec(),
            drivetrainConfig.getMaxAngularVelocityRadiansPerSec()
        );
        Logger.recordOutput("SwerveDrive/obtainableFieldRelativeSpeeds", obtainableFieldRelativeSpeeds);

        ChassisSpeeds obtainableRobotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(obtainableFieldRelativeSpeeds, RobotState.getInstance().getEstimatedPose().getRotation());
        Logger.recordOutput("SwerveDrive/obtainableRobotRelativeSpeeds", obtainableRobotRelativeSpeeds);

        SwerveModuleState[] moduleSetpoints = kinematics.toSwerveModuleStates(obtainableRobotRelativeSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
            moduleSetpoints, 
            drivetrainConfig.getMaxModuleVelocity()
        );
        Logger.recordOutput("SwerveDrive/desaturatedModuleSetpoints", moduleSetpoints);

        for (int i = 0; i < 4; i++) {
            moduleSetpoints[i].optimize(moduleStates[i].angle);
            modules[i].setState(moduleSetpoints[i]);
        }
        Logger.recordOutput("SwerveDrive/optimizedModuleSetpoints", moduleSetpoints);
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, RobotState.getInstance().getEstimatedPose().getRotation());
        driveRobotRelative(speeds);
    }

    public void resetGyro(Rotation2d yaw) {
        gyroIO.resetGyro(yaw);
    }

    public Rotation2d getGyroAngle() {
        return gyroInputs.yawPosition;
    }

    public void setWheelCoast(boolean isCoast) {
        for (ModuleIO module : modules) {
            module.setWheelCoast(isCoast);
        }
    }

    public Command getDynamicDriveCharacterizationSysIdRoutine(Direction direction) {
        return driveCharacterizationSysIdRoutine.dynamic(direction);
    }

    public Command getDynamicSteerCharacterizationSysIdRoutine(Direction direction) {
        return steerCharacterizationSysIdRoutine.dynamic(direction);
    }

    public Command getQuasistaticDriveCharacterizationSysIdRoutine(Direction direction) {
        return driveCharacterizationSysIdRoutine.quasistatic(direction);
    }

    public Command getQuasistaticSteerCharacterizationSysIdRoutine(Direction direction) {
        return steerCharacterizationSysIdRoutine.quasistatic(direction);
    }

    public FollowPath.Builder getFollowPathBuilder() {
        return followPathBuilder;
    }
}

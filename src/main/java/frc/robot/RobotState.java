package frc.robot;

import edu.wpi.first.math.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.constants.robotState.RobotStateConfigBase;
import frc.robot.constants.robotState.RobotStateConfigProto;
import frc.robot.constants.robotState.RobotStateConfigSim;
import frc.robot.constants.robotState.RobotStateConfigComp;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigBase;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigComp;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigProto;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigSim;

import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.function.Consumer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState {
    private static RobotState instance;
    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }
  
    public record OdometryObservation(
        double timestampsSeconds,
        boolean isGyroConnected,
        SwerveModulePosition[] modulePositions, 
        SwerveModuleState[] moduleStates,
        Rotation2d yawPosition,
        double yawVelocityRadPerSec
    ) {}

    public static enum VisionObservationScale {
        GLOBAL,
        LOCAL
    }

    public record VisionObservation(
        Pose2d visionPose, 
        double timestamp, 
        Matrix<N3, N1> stdDevs,
        VisionObservationScale scale
    ) {}


    private static final double poseBufferSizeSeconds = 2.0;
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);

    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    private double lastEstimatedPoseUpdateTime = 0;
    private int localVisionObservationUpdateCount = 0;

    // Odometry
    private final SwerveDriveKinematics kinematics;
    private SwerveModulePosition[] lastWheelPositions = {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    private double lastYawVelocityRadPerSec = 0;
    private ChassisSpeeds lastRobotRelativeSpeeds = new ChassisSpeeds();

    private final SwerveDrivetrainConfigBase drivetrainConfig;
    private final RobotStateConfigBase robotStateConfig;

    private final ArrayList<Runnable> onOdometryUpdateRunnables = new ArrayList<Runnable>();
    private final ArrayList<Consumer<Translation2d>> onLocalVisionEstimateRunnables = new ArrayList<Consumer<Translation2d>>();
    private final ArrayList<Runnable> onGlobalVisionEstimateRunnables = new ArrayList<Runnable>();

    private VisionObservationScale requestedObservationScale = VisionObservationScale.GLOBAL;

    private RobotState() {
        switch (Constants.currentMode) {
            case COMP:
                drivetrainConfig = SwerveDrivetrainConfigComp.getInstance();
                robotStateConfig = RobotStateConfigComp.getInstance();

                break;

            case PROTO:
                drivetrainConfig = SwerveDrivetrainConfigProto.getInstance();
                robotStateConfig = RobotStateConfigProto.getInstance();

                break;
            
            case SIM:
                drivetrainConfig = SwerveDrivetrainConfigSim.getInstance();
                robotStateConfig = RobotStateConfigSim.getInstance();

                break;

            case REPLAY:
                drivetrainConfig = SwerveDrivetrainConfigComp.getInstance();
                robotStateConfig = RobotStateConfigComp.getInstance();

                break;

            default:
                drivetrainConfig = SwerveDrivetrainConfigComp.getInstance();
                robotStateConfig = RobotStateConfigComp.getInstance();


                break;
    }

        kinematics = new SwerveDriveKinematics(
            drivetrainConfig.getFrontLeftPositionMeters(),
            drivetrainConfig.getFrontRightPositionMeters(),
            drivetrainConfig.getBackLeftPositionMeters(),
            drivetrainConfig.getBackRightPositionMeters()
        ); 

        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            new Rotation2d(), 
            lastWheelPositions, 
            new Pose2d(),
            VecBuilder.fill(
                robotStateConfig.getOdomTranslationDevBase(),
                robotStateConfig.getOdomTranslationDevBase(),
                0
            ),
            VecBuilder.fill(
                robotStateConfig.getVisionTranslationDevBase(),
                robotStateConfig.getVisionTranslationDevBase(),
                9999999
            )
        );  
    }   

    /** Add odometry observation */
    public void addOdometryObservation(OdometryObservation observation) {
        Logger.recordOutput("RobotState/odometry/timestamp", observation.timestampsSeconds());
        Logger.recordOutput("RobotState/odometry/isGyroConnected", observation.isGyroConnected());
        Logger.recordOutput("RobotState/odometry/modulePositions", observation.modulePositions());
        Logger.recordOutput("RobotState/odometry/moduleStates", observation.moduleStates());
        Logger.recordOutput("RobotState/odometry/yawPosition", observation.yawPosition());
        Logger.recordOutput("RobotState/odometry/yawVelocityRadPerSec", observation.yawVelocityRadPerSec());

        // update robotState member variables
        lastRobotRelativeSpeeds = kinematics.toChassisSpeeds(observation.moduleStates);
        lastRobotRelativeSpeeds.omegaRadiansPerSecond = observation.isGyroConnected ? observation.yawVelocityRadPerSec() : lastRobotRelativeSpeeds.omegaRadiansPerSecond;
        lastYawVelocityRadPerSec = observation.isGyroConnected ? observation.yawVelocityRadPerSec() : lastRobotRelativeSpeeds.omegaRadiansPerSecond;

        swerveDrivePoseEstimator.updateWithTime(
            observation.timestampsSeconds(), 
            observation.isGyroConnected() ? 
                observation.yawPosition() : 
                new Rotation2d(
                    swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getRadians() + 
                    kinematics.toTwist2d(lastWheelPositions, observation.modulePositions()).dtheta
                ), 
            observation.modulePositions()
        );
        lastWheelPositions = observation.modulePositions();

        lastEstimatedPoseUpdateTime = Timer.getTimestamp();

        // Add pose to buffer at timestamp
        poseBuffer.addSample(lastEstimatedPoseUpdateTime, swerveDrivePoseEstimator.getEstimatedPosition()); 

        for (Runnable runnable : onOdometryUpdateRunnables) {
            runnable.run();
        }

        Logger.recordOutput("RobotState/vision/localVisionObservationUpdateCount", localVisionObservationUpdateCount);
        Logger.recordOutput("RobotState/vision/requestedObservationScale", requestedObservationScale);
    }

    public void addVisionObservation(VisionObservation observation) {
        // If measurement is old enough to be outside the pose buffer's timespan, skip.
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSeconds > observation.timestamp()) {
                return;
            }
        } 
        
        catch (NoSuchElementException ex) {
            return;
        }

        Logger.recordOutput("RobotState/vision/stdDevTranslation", observation.stdDevs().get(0,0));
        Logger.recordOutput("RobotState/vision/visionPose", observation.visionPose());

        swerveDrivePoseEstimator.addVisionMeasurement(observation.visionPose(), observation.timestamp(), observation.stdDevs());
        if (observation.scale() == VisionObservationScale.LOCAL && requestedObservationScale == VisionObservationScale.LOCAL) {
            localVisionObservationUpdateCount++;
        }
        else if (requestedObservationScale == VisionObservationScale.GLOBAL) {
            localVisionObservationUpdateCount = 0;
        }

        lastEstimatedPoseUpdateTime = Timer.getTimestamp();
    }


    public void registerRunnableOnOdometryUpdate(Runnable runnable) {
        onOdometryUpdateRunnables.add(runnable);
    }

    public void registerRunnableOnLocalVisionEstimateRequest(Consumer<Translation2d> runnable) {
        onLocalVisionEstimateRunnables.add(runnable);
    }

    public void registerRunnableOnGlobalVisionEstimateRequest(Runnable runnable) {
        onGlobalVisionEstimateRunnables.add(runnable);
    }

    public void requestGlobalVisionEstimateScale() {
        for (Runnable runnable : onGlobalVisionEstimateRunnables) {
            runnable.run();
        }

        requestedObservationScale = VisionObservationScale.GLOBAL;
        localVisionObservationUpdateCount = 0;
    }

    public void requestLocalVisionEstimateScale(Translation2d pose) {
        for (Consumer<Translation2d> runnable : onLocalVisionEstimateRunnables) {
            runnable.accept(pose);
        }

        requestedObservationScale = VisionObservationScale.LOCAL;
    }  

    /**
     * Reset estimated pose and odometry pose to pose <br>
     * Clear pose buffer
     */
    public void resetPose(Pose2d initialPose) {
        // SwerveDrive.getInstance().resetGyro(initialPose.getRotation());
        swerveDrivePoseEstimator.resetPosition(initialPose.getRotation(), lastWheelPositions, initialPose);

        poseBuffer.clear();
    }

    public void zeroGyro() {
        resetPose(new Pose2d(getEstimatedPose().getTranslation(), new Rotation2d()));
    }

    @AutoLogOutput(key = "RobotState/isPoseEstimateValid")
    public boolean isPoseEstimateValid() {
        if (requestedObservationScale == VisionObservationScale.LOCAL) {
            return localVisionObservationUpdateCount >= robotStateConfig.getMinLocalVisionObservationCount();
        }
        return true;
    }

    @AutoLogOutput(key = "RobotState/estimatedPose")
    public Pose2d getEstimatedPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public double getYawVelocityRadPerSec() {
        return lastYawVelocityRadPerSec;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return lastRobotRelativeSpeeds;
    }

    @AutoLogOutput(key = "RobotState/fieldRelativeSpeeds")
    public ChassisSpeeds getFieldRelativeSpeeds() { 
        return ChassisSpeeds.fromRobotRelativeSpeeds(lastRobotRelativeSpeeds, getEstimatedPose().getRotation());
    }

    public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
        return getEstimatedPose()
            .transformBy(
                new Transform2d(
                    lastRobotRelativeSpeeds.vxMetersPerSecond * translationLookaheadS,
                    lastRobotRelativeSpeeds.vyMetersPerSecond * translationLookaheadS,
                    Rotation2d.fromRadians(lastRobotRelativeSpeeds.omegaRadiansPerSecond * rotationLookaheadS)
                )
            );
    }

    public Pose2d getPredictedPose(double timestamp) {
        return getPredictedPose(timestamp - lastEstimatedPoseUpdateTime, timestamp - lastEstimatedPoseUpdateTime);
    }
}
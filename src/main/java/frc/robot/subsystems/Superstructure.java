package frc.robot.subsystems;

import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.lib.util.ShotCalculator;
import frc.robot.lib.util.ShotCalculator.ShotData;
import frc.robot.subsystems.shooter.Shooter;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance;
    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    public enum CurrentSuperstructureState {
        DISABLED,
        IDLE,
        TRACKING_IDLE,
        TRACKING_WINDUP,
        TRACKING_READY,
        SHOOTING
    }
    public enum DesiredSuperstructureState {
        DISABLED,
        IDLE,
        TRACKING_IDLE,
        TRACKING_READY,
        SHOOTING
    }

    private CurrentSuperstructureState currentSuperstructureState = CurrentSuperstructureState.DISABLED;
    private DesiredSuperstructureState desiredSuperstructureState = DesiredSuperstructureState.DISABLED;

    private final Shooter shooter = Shooter.getInstance();
    private final RobotState robotState = RobotState.getInstance();

    private Superstructure() {}

    @Override
    public void periodic() {
        switch (currentSuperstructureState) {
            case DISABLED:
                switch (desiredSuperstructureState) {
                    case DISABLED:
                        handleDisabledState();
                        break;
                    case IDLE:
                        handleIdleState();
                        break;
                    case TRACKING_IDLE:
                        handleTrackingIdleState();
                        break;
                    case TRACKING_READY:
                        handleTrackingReadyState();
                        break;
                    case SHOOTING:
                        handleTrackingWindupState();
                        break;
                }
                break;

            case IDLE:
                switch (desiredSuperstructureState) {
                    case DISABLED:
                        handleDisabledState();
                        break;
                    case IDLE:
                        handleIdleState();
                        break;
                    case TRACKING_IDLE:
                        handleTrackingIdleState();
                        break;
                    case TRACKING_READY:
                        handleTrackingReadyState();
                        break;
                    case SHOOTING:
                        handleTrackingWindupState();
                        break;
                }
                break;

            case TRACKING_IDLE:
                switch (desiredSuperstructureState) {
                    case DISABLED:
                        handleDisabledState();
                        break;
                    case IDLE:
                        handleIdleState();
                        break;
                    case TRACKING_IDLE:
                        handleTrackingIdleState();
                        break;
                    case TRACKING_READY:
                        handleTrackingReadyState();
                        break;
                    case SHOOTING:
                        handleTrackingWindupState();
                        break;
                }
                break;

            case TRACKING_WINDUP:
                switch (desiredSuperstructureState) {
                    case DISABLED:
                        handleDisabledState();
                        break;
                    case IDLE:
                        handleIdleState();
                        break;
                    case TRACKING_IDLE:
                        handleTrackingIdleState();
                        break;
                    case TRACKING_READY:
                        handleTrackingReadyState();
                        break;
                    case SHOOTING:
                        handleTrackingWindupState();
                        break;
                }
                break;

            case TRACKING_READY:
                switch (desiredSuperstructureState) {
                    case DISABLED:
                        handleDisabledState();
                        break;
                    case IDLE:
                        handleIdleState();
                        break;
                    case TRACKING_IDLE:
                        handleTrackingIdleState();
                        break;
                    case TRACKING_READY:
                        handleTrackingReadyState();
                        break;
                    case SHOOTING:
                        handleShootingState();
                        break;
                }
                break;

            case SHOOTING:
                // Stay in shooting until shot completes, then returns to TRACKING_READY
                handleShootingState();
                break;
        }
    }

    private void handleDisabledState() {
        currentSuperstructureState = CurrentSuperstructureState.DISABLED;

        //TODO: Switch motors to coast or something TBD
    }

    private void handleIdleState() {
        currentSuperstructureState = CurrentSuperstructureState.IDLE;

        shooter.setShotVelocity(0);
        shooter.setFeedVelocity(0);
        shooter.setIndexerVelocity(0);
        shooter.setHoodAngle(Rotation2d.fromDegrees(45.0));
        shooter.setTurretAngle(new Rotation2d(0));
    }

    private void handleTrackingIdleState() {
        currentSuperstructureState = CurrentSuperstructureState.TRACKING_IDLE;

        ShotData shotData = calculateShotData();
        Rotation2d turretShotYaw = shotData.targetFieldYaw().minus(robotState.getEstimatedPose().getRotation());

        shooter.setShotVelocity(0);
        shooter.setFeedVelocity(0);
        shooter.setIndexerVelocity(0);
        shooter.setHoodAngle(shotData.hoodPitch());
        shooter.setTurretAngle(turretShotYaw);
    }

    private void handleTrackingWindupState() {
        currentSuperstructureState = CurrentSuperstructureState.TRACKING_WINDUP;

        ShotData shotData = calculateShotData();
        Rotation2d turretShotYaw = shotData.targetFieldYaw().minus(robotState.getEstimatedPose().getRotation());

        shooter.setShotVelocity(shotData.flywheelRPS());
        shooter.setFeedVelocity(35);
        shooter.setIndexerVelocity(0);
        shooter.setHoodAngle(shotData.hoodPitch());
        shooter.setTurretAngle(turretShotYaw);

        if (
            shooter.isFlywheelAtSetpoint() && 
            shooter.isFeederAtSetpoint() && 
            shooter.isHoodAtSetpoint() && 
            shooter.isTurretAtSetpoint() 
            //TODO: AND ROBOT STATE READY TO SHOOT (swerve is within tolerance for the turret yaw)

        ) {
            currentSuperstructureState = CurrentSuperstructureState.TRACKING_READY;
        }
    }

    private void handleTrackingReadyState() {
        currentSuperstructureState = CurrentSuperstructureState.TRACKING_READY;

        ShotData shotData = calculateShotData();
        Rotation2d turretShotYaw = shotData.targetFieldYaw().minus(robotState.getEstimatedPose().getRotation());

        shooter.setShotVelocity(shotData.flywheelRPS());
        shooter.setFeedVelocity(35);
        shooter.setIndexerVelocity(0);
        shooter.setHoodAngle(shotData.hoodPitch());
        shooter.setTurretAngle(turretShotYaw);

        // If setpoints are lost, fall back to TRACKING_WINDUP
        if (!(
            shooter.isFlywheelAtSetpoint() && 
            shooter.isFeederAtSetpoint() && 
            shooter.isHoodAtSetpoint() && 
            shooter.isTurretAtSetpoint()
            //TODO: AND ROBOT STATE READY TO SHOOT (swerve is within tolerance for the turret yaw)
        )) {
            currentSuperstructureState = CurrentSuperstructureState.TRACKING_WINDUP;
        }
    }

    private void handleShootingState() {
        currentSuperstructureState = CurrentSuperstructureState.SHOOTING;

        ShotData shotData = calculateShotData();
        Rotation2d turretShotYaw = shotData.targetFieldYaw().minus(robotState.getEstimatedPose().getRotation());

        shooter.setShotVelocity(shotData.flywheelRPS());
        shooter.setFeedVelocity(35);
        shooter.setIndexerVelocity(35);
        shooter.setHoodAngle(shotData.hoodPitch());
        shooter.setTurretAngle(turretShotYaw);

        //TODO: Detect when note has left (beam break, flywheel velocity drop, or timer)
        // For now, transition back to TRACKING_READY immediately after shot cycle
        // In practice, you'd check: if (shooter.hasNoteExited()) { ... }
        currentSuperstructureState = CurrentSuperstructureState.TRACKING_READY;
    }

    private ShotData calculateShotData() {
        Translation3d targetLocation = Constants.FieldConstants.kSHOOTER_TARGET;
        Pose3d shooterPose = 
            new Pose3d(
                new Translation3d(
                    robotState.getEstimatedPose().getX(), 
                    robotState.getEstimatedPose().getY(), 
                    0
                ),
                new Rotation3d(0, 0, robotState.getEstimatedPose().getRotation().getRadians())
            ).plus(new Transform3d(new Pose3d(), shooter.getShooterRelativePose()));
        ChassisSpeeds speeds = robotState.getFieldRelativeSpeeds();
        InterpolatingMatrixTreeMap<Double, N2, N1> lerpTable = shooter.getLerpTable();
        double latencyCompensationSeconds = shooter.getLatencyCompensationSeconds();
        DoubleUnaryOperator rpsToExitVelocity = shooter::calculateShotExitVelocityMetersPerSec;

        return ShotCalculator.calculate(
            targetLocation,
            shooterPose,
            speeds,
            lerpTable,
            latencyCompensationSeconds,
            rpsToExitVelocity
        );
    }

    public void setDesiredSuperstructureState(DesiredSuperstructureState desiredSuperstructureState) {
        this.desiredSuperstructureState = desiredSuperstructureState;
    }

    public CurrentSuperstructureState getCurrentSuperstructureState() {
        return currentSuperstructureState;
    }

    public DesiredSuperstructureState getDesiredSuperstructureState() {
        return desiredSuperstructureState;
    }
}
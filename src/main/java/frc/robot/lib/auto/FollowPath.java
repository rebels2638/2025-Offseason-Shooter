package frc.robot.lib.auto;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.lib.auto.Path.TranslationTarget;
import frc.robot.lib.auto.Path.RotationTarget;
import frc.robot.subsystems.swerve.SwerveDrive;

public class FollowPath extends Command {
    
    private SwerveDrive swerve = SwerveDrive.getInstance();
    private RobotState robotState = RobotState.getInstance();
    private Pose2d currentPose = new Pose2d();

    private final double defaultMaxTranslationalVelocity = 4.5; // m/s
    private final double defaultMaxRotationalVelocity = 6; // rad/s
    private double currentMaxTranslationalVelocity = defaultMaxTranslationalVelocity;
    private double currentMaxRotationalVelocity = defaultMaxRotationalVelocity;
    private final double defaultMaxTranslationalAcceleration = 15;// m/s^2
    private final double defaultMaxRotationalAcceleration = 5; // rad/s^2
    private final SlewRateLimiter translationalSlewRateLimiter = new SlewRateLimiter(defaultMaxTranslationalAcceleration); // m/s^2
    private final SlewRateLimiter rotationalSlewRateLimiter = new SlewRateLimiter(defaultMaxRotationalAcceleration); // rad/s^2

    private final PIDController translationController = new PIDController(4, 0, 0);
    private final PIDController rotationController = new PIDController(3, 0, 0.3);
    private boolean translationClosedLoop = false;
    
    private final Path path; 
    private final List<Translation2d> translationTargets;
    private final List<TranslationTarget> translationPathElements;
    private final List<Rotation2d> rotationTargets;
    private final List<RotationTarget> rotationPathElements;
    private final List<Boolean> translationTargetWaypointOverlap;

    private int currentTranslationTargetIndex = 0;
    private int currentRotationTargetIndex = 0;

    private Rotation2d targetRotation = new Rotation2d();
    private Translation2d targetTranslation = new Translation2d();

    private final double defaultIntermediateHandoffRadius = 0.23; // meters
    private double currentIntermediateHandoffRadius = defaultIntermediateHandoffRadius; // meters

    private final double translationEndTolerance = 0.05; // meters
    private final double rotationEndTolerance = Math.toRadians(5); // radians

    private ArrayList<Translation2d> robotTranslations = new ArrayList<>();

    private double cyclesSinceLog = 0;

    public FollowPath(Path path) {
        this.path = path;
        this.translationTargets = path.getTranslationTargets();
        this.translationPathElements = path.getTranslationPathElements();
        this.rotationTargets = path.getRotationTargets();
        this.rotationPathElements = path.getRotationPathElements();
        this.translationTargetWaypointOverlap = path.getTranslationTargetWaypointOverlap();

        translationController.setTolerance(translationEndTolerance);
        rotationController.setTolerance(rotationEndTolerance);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        currentPose = robotState.getEstimatedPose();

        currentTranslationTargetIndex = 0;
        currentRotationTargetIndex = 0;

        targetRotation = robotState.getEstimatedPose().getRotation();
        targetTranslation = path.getTranslationTargets().get(0);

        rotationController.reset();
        translationController.reset();
        translationClosedLoop = false;

        translationalSlewRateLimiter.reset(Math.hypot(robotState.getFieldRelativeSpeeds().vxMetersPerSecond, robotState.getFieldRelativeSpeeds().vyMetersPerSecond));
        rotationalSlewRateLimiter.reset(robotState.getFieldRelativeSpeeds().omegaRadiansPerSecond);

        Logger.recordOutput("FollowPath/translationWaypoints", path.getTranslationTargets().toArray(new Translation2d[0]));

        robotTranslations = new ArrayList<>();
    }

    @Override
    public void execute() {
        currentPose = robotState.getEstimatedPose();

        // translational velo
        double translationalVelocity;
        targetTranslation = translationTargets.get(currentTranslationTargetIndex);
        TranslationTarget targetTranslationPathElement = translationPathElements.get(currentTranslationTargetIndex);
        translationClosedLoop = false;

        // handoff to next waypoint if it is intermediate and close
        if (Math.hypot(currentPose.getX() - targetTranslation.getX(), currentPose.getY() - targetTranslation.getY()) <= currentIntermediateHandoffRadius && currentTranslationTargetIndex < translationTargets.size()-1) {
            currentTranslationTargetIndex++;
            targetTranslation = translationTargets.get(currentTranslationTargetIndex);
            targetTranslationPathElement = translationPathElements.get(currentTranslationTargetIndex);
        }
        
        if (targetTranslationPathElement.maxAccelerationMetersSPerSec2().isPresent()) {
            translationalSlewRateLimiter.setRateLimit(targetTranslationPathElement.maxAccelerationMetersSPerSec2().get().doubleValue());
        }
        else {
            translationalSlewRateLimiter.setRateLimit(defaultMaxTranslationalAcceleration);
        }
        if (targetTranslationPathElement.maxVelocityMetersPerSec().isPresent()) {
            currentMaxTranslationalVelocity = targetTranslationPathElement.maxVelocityMetersPerSec().get().doubleValue();
        }
        else {
            currentMaxTranslationalVelocity = defaultMaxTranslationalVelocity;
        }
        if (targetTranslationPathElement.intermediateHandoffRadiusMeters().isPresent()) {
            currentIntermediateHandoffRadius = targetTranslationPathElement.intermediateHandoffRadiusMeters().get().doubleValue();
        }
        else {
            currentIntermediateHandoffRadius = defaultIntermediateHandoffRadius;
        }
        
        // b-line straight to intermediate 
        if (currentTranslationTargetIndex < translationTargets.size()-1 || (targetTranslationPathElement.finalVelocityMetersPerSec().isPresent() && targetTranslationPathElement.finalVelocityMetersPerSec().get().doubleValue() != 0.0)) {
            translationalVelocity = translationalSlewRateLimiter.calculate(
                MathUtil.clamp(
                    targetTranslationPathElement.finalVelocityMetersPerSec().isPresent() ? 
                        targetTranslationPathElement.finalVelocityMetersPerSec().get().doubleValue() : 
                        currentMaxTranslationalVelocity,
                    -currentMaxTranslationalVelocity,
                    currentMaxTranslationalVelocity
                )
            );
        }
        // PID to end point
        else {
            double distanceToTarget = Math.hypot(currentPose.getX() - targetTranslation.getX(), currentPose.getY() - targetTranslation.getY());
            translationalVelocity = 
                translationalSlewRateLimiter.calculate(
                    MathUtil.clamp(
                        -translationController.calculate(distanceToTarget, 0),
                        -currentMaxTranslationalVelocity,
                        currentMaxTranslationalVelocity
                    )
                );
            translationClosedLoop = true;
        }
        double angleToTarget = Math.atan2(
            targetTranslation.getY() - currentPose.getTranslation().getY(), 
            targetTranslation.getX() - currentPose.getTranslation().getX()
        );

        // rotational velo
        if (rotationTargets.size() > 0 && currentRotationTargetIndex < rotationTargets.size()-1) {
            targetRotation = rotationTargets.get(currentRotationTargetIndex);
            RotationTarget targetRotationPathElement = rotationPathElements.get(currentRotationTargetIndex);
    
            if ((currentPose.getTranslation().getDistance(rotationPathElements.get(currentRotationTargetIndex+1).translation())
                < targetRotationPathElement.translation().getDistance(rotationPathElements.get(currentRotationTargetIndex+1).translation())) ||
                translationClosedLoop && translationController.atSetpoint() && rotationController.atSetpoint()) { // in case robot is stationary 
                    // and cant switch last targets after translation targets are exhausted
                currentRotationTargetIndex++;
                targetRotation = rotationTargets.get(currentRotationTargetIndex);
                targetRotationPathElement = rotationPathElements.get(currentRotationTargetIndex);
            }
    
            if (targetRotationPathElement.maxAccelerationRadSPerSec2().isPresent()) {
                rotationalSlewRateLimiter.setRateLimit(targetRotationPathElement.maxAccelerationRadSPerSec2().get().doubleValue());
            }
            else {
                rotationalSlewRateLimiter.setRateLimit(defaultMaxRotationalAcceleration);
            }
            if (targetRotationPathElement.maxVelocityRadPerSec().isPresent()) {
                currentMaxRotationalVelocity = targetRotationPathElement.maxVelocityRadPerSec().get().doubleValue();
            }
            else {
                currentMaxRotationalVelocity = defaultMaxRotationalVelocity;
            }
        }

        double rotationalVelocity = 
            rotationalSlewRateLimiter.calculate(
                MathUtil.clamp(
                    rotationController.calculate(currentPose.getRotation().getRadians(), targetRotation.getRadians()),
                    -currentMaxRotationalVelocity,
                    currentMaxRotationalVelocity
                )
            );
        

        ChassisSpeeds speeds = new ChassisSpeeds(
            translationalVelocity * Math.cos(angleToTarget),
            translationalVelocity * Math.sin(angleToTarget),
            rotationalVelocity
        );

        swerve.driveFieldRelative(speeds);

        if (cyclesSinceLog++ % 5 == 0) {
            robotTranslations.add(currentPose.getTranslation());
        }

        Logger.recordOutput("FollowPath/robotTranslations", robotTranslations.toArray(Translation2d[]::new));
        Logger.recordOutput("FollowPath/targetTranslation", targetTranslation);
        Logger.recordOutput("FollowPath/targetRotation", targetRotation);
        Logger.recordOutput("FollowPath/combinedTargetPose", new Pose2d(targetTranslation, targetRotation));
        Logger.recordOutput("FollowPath/setSpeeds", speeds);
    }

    @Override
    public boolean isFinished() {
        return 
            (currentTranslationTargetIndex == translationTargets.size()-1 && translationController.atSetpoint() && translationClosedLoop && 
            currentRotationTargetIndex == rotationTargets.size()-1 && rotationController.atSetpoint()) || // zero final end velo
            (currentTranslationTargetIndex == translationTargets.size()-1 && !translationClosedLoop && 
            currentPose.getTranslation().getDistance(targetTranslation) <= currentIntermediateHandoffRadius); // non-zero final end velo. 
            // assume that there will be a proceeding command that further aligns the robot, so accuracy is not as important as speed / timing
    }
}
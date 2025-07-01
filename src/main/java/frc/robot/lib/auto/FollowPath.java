package frc.robot.lib.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.SwerveDrive;

public class FollowPath extends Command {
    public static record Waypoint(Translation2d translation, Optional<Rotation2d> rotation, Optional<Double> velocity) {}
    
    private SwerveDrive swerve = SwerveDrive.getInstance();
    private RobotState robotState = RobotState.getInstance();

    private final SlewRateLimiter translationalSlewRateLimiter = new SlewRateLimiter(15); // m/s^2
    private final SlewRateLimiter rotationalSlewRateLimiter = new SlewRateLimiter(5); // rad/s^2
    private final double maxTranslationalVelocity = 4.5; // m/s
    private final double maxRotationalVelocity = 6; // rad/s

    private final PIDController translationController = new PIDController(4, 0, 0);
    private final PIDController rotationController = new PIDController(3, 0, 0.3);

    private List<Waypoint> waypoints;
    private int currentWaypointIndex = 0;

    private Rotation2d targetRotation = new Rotation2d();
    private Translation2d targetTranslation = new Translation2d();

    private final double intermediateHandoffRadius = 0.23; // meters
    private final double translationEndTolerance = 0.05; // meters
    private final double rotationEndTolerance = Math.toRadians(5); // radians

    private ArrayList<Translation2d> robotTranslations = new ArrayList<>();

    public FollowPath(List<Waypoint> waypoints) {
        this.waypoints = waypoints;

        translationController.setTolerance(translationEndTolerance);
        rotationController.setTolerance(rotationEndTolerance);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        currentWaypointIndex = 0;
        targetRotation = robotState.getEstimatedPose().getRotation();
        targetTranslation = waypoints.get(0).translation;

        rotationController.reset();
        translationController.reset();

        Translation2d[] translationWaypoints = new Translation2d[waypoints.size()];
        for (int i = 0; i < waypoints.size(); i++) {
            translationWaypoints[i] = waypoints.get(i).translation;
        }
        Logger.recordOutput("FollowPath/translationWaypoints", translationWaypoints);

        robotTranslations = new ArrayList<>();
    }

    @Override
    public void execute() {
        Pose2d currentPose = robotState.getEstimatedPose();

        // translational velo
        double translationVelocity;
        targetTranslation = waypoints.get(currentWaypointIndex).translation;
        // handoff to next waypoint if it is intermediate and close
        if (Math.hypot(currentPose.getX() - targetTranslation.getX(), currentPose.getY() - targetTranslation.getY()) <= intermediateHandoffRadius && currentWaypointIndex < waypoints.size()-1) {
            currentWaypointIndex++;
            targetTranslation = waypoints.get(currentWaypointIndex).translation;
        }
        // b-line straight to intermediate 
        if (currentWaypointIndex < waypoints.size()-1) {
            translationVelocity = translationalSlewRateLimiter.calculate(
                waypoints.get(currentWaypointIndex).velocity.isPresent() ? waypoints.get(currentWaypointIndex).velocity.get() : maxTranslationalVelocity
            );
        }
        // PID to end point
        else {
            double distanceToTarget = Math.hypot(currentPose.getX() - targetTranslation.getX(), currentPose.getY() - targetTranslation.getY());
            translationVelocity = 
                translationalSlewRateLimiter.calculate(
                    MathUtil.clamp(
                        translationController.calculate(0, distanceToTarget),
                        -maxTranslationalVelocity,
                        maxTranslationalVelocity
                    )
                );
        }
        double angleToTarget = Math.atan2(
            targetTranslation.getY() - currentPose.getTranslation().getY(), 
            targetTranslation.getX() - currentPose.getTranslation().getX()
        );

        // rotational velo
        for (int i = currentWaypointIndex; i < waypoints.size(); i++) {
            if (waypoints.get(i).rotation.isPresent()) {
                targetRotation = waypoints.get(i).rotation.get();
                break;
            }
        }
        double rotationVelocity = 
            rotationalSlewRateLimiter.calculate(
                MathUtil.clamp(
                    rotationController.calculate(currentPose.getRotation().getRadians(), targetRotation.getRadians()),
                    -maxRotationalVelocity,
                    maxRotationalVelocity
                )
            );
        
        ChassisSpeeds speeds = new ChassisSpeeds(
            translationVelocity * Math.cos(angleToTarget),
            translationVelocity * Math.sin(angleToTarget),
            rotationVelocity
        );

        swerve.driveFieldRelative(speeds);

        robotTranslations.add(currentPose.getTranslation());

        Logger.recordOutput("FollowPath/robotTranslations", robotTranslations.toArray(Translation2d[]::new));
        Logger.recordOutput("FollowPath/targetTranslation", targetTranslation);
        Logger.recordOutput("FollowPath/targetRotation", targetRotation);
        Logger.recordOutput("FollowPath/combinedTargetPose", new Pose2d(targetTranslation, targetRotation));
        Logger.recordOutput("FollowPath/setSpeeds", speeds);
        Logger.recordOutput("FollowPath/pidRot", rotationController.calculate(currentPose.getRotation().getRadians(), targetRotation.getRadians()));
    }

    @Override
    public boolean isFinished() {
        return currentWaypointIndex == waypoints.size()-1 && translationController.atSetpoint() && rotationController.atSetpoint();
    }
}
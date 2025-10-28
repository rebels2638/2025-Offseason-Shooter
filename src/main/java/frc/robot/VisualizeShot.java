package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.util.ProjectileVisualizer;
import frc.robot.subsystems.shooter.Shooter;

public class VisualizeShot {
    public VisualizeShot() {
        // Convert robot pose from field-relative Pose2d to Pose3d
        Pose3d robotPose3d = new Pose3d(RobotState.getInstance().getEstimatedPose());
        
        // Transform shooter relative pose to field coordinates
        Pose3d shooterPose = robotPose3d.plus(
            new Transform3d(new Pose3d(), Shooter.getInstance().getShooterRelativePose())
        );

        // Apply hood angle as a local rotation (pitch around the shooter's Y-axis)
        // This needs to be done by creating a proper rotation that combines:
        // 1. The robot's yaw (Z rotation) - already in shooterPose
        // 2. The hood's pitch (Y rotation) - needs to be added
        Rotation3d currentRotation = shooterPose.getRotation();
        double hoodAngleRadians = Shooter.getInstance().getHoodAngleRotations() * (2 * Math.PI);
        
        // Create new rotation: keep the yaw (Z), add pitch (Y), keep roll (X) at 0
        Rotation3d newRotation = new Rotation3d(
            0,  // roll (X)
            hoodAngleRadians,  // pitch (Y) 
            currentRotation.getZ()  // yaw (Z) from robot rotation
        );
        
        shooterPose = new Pose3d(shooterPose.getTranslation(), newRotation);

        Logger.recordOutput("VisualizeShot/shooterPose", shooterPose);
        Logger.recordOutput("VisualizeShot/launchTime", Timer.getFPGATimestamp());

        new ProjectileVisualizer(
            RobotState.getInstance().getFieldRelativeSpeeds().vxMetersPerSecond,
            RobotState.getInstance().getFieldRelativeSpeeds().vyMetersPerSecond,
            Shooter.getInstance().getShotExitVelocityMetersPerSec(),
            shooterPose
        ).schedule();
    }
}

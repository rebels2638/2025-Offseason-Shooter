package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

        shooterPose = new Pose3d(shooterPose.getTranslation(), shooterPose.getRotation().plus(
            new Rotation3d(0, Shooter.getInstance().getHoodAngleRotations() * (2 * Math.PI), 0)
        ));

        Logger.recordOutput("VisualizeShot/shooterPose", shooterPose);
        new ProjectileVisualizer(
            RobotState.getInstance().getFieldRelativeSpeeds().vxMetersPerSecond,
            RobotState.getInstance().getFieldRelativeSpeeds().vyMetersPerSecond,
            Shooter.getInstance().getShotExitVelocityMetersPerSec(),
            shooterPose
        ).schedule();
    }
}

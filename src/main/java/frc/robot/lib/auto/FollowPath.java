package frc.robot.lib.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.auto.Path.PathElement;
import frc.robot.lib.auto.Path.RotationTarget;
import frc.robot.lib.auto.Path.TranslationTarget;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class FollowPath extends Command {
    private static PIDController translationController = null;
    private static PIDController rotationController = null;

    public static PIDController getTranslationController() { 
        return new PIDController(translationController.getP(), translationController.getI(), translationController.getD()); 
    }
    public static PIDController getRotationController() { 
        return new PIDController(rotationController.getP(), rotationController.getI(), rotationController.getD()); 
    }
    public static void setTranslationController(PIDController translationController) { 
        if (translationController == null) {
            throw new IllegalArgumentException("Translation controller must not be null");
        }
        FollowPath.translationController = new PIDController(translationController.getP(), translationController.getI(), translationController.getD());
    }
    public static void setRotationController(PIDController rotationController) {
        if (rotationController == null) {
            throw new IllegalArgumentException("Rotation controller must not be null");
        }
        FollowPath.rotationController = new PIDController(rotationController.getP(), rotationController.getI(), rotationController.getD());
    }

    private final Path path;
    private final Supplier<Pose2d> poseSupplier;
    private final Consumer<ChassisSpeeds> robotRelativeSpeedsConsumer;

    private int segmentIndex = 0;
    private int rotationElementIndex = 0;

    private List<List<PathElement>> pathSegments;

    public FollowPath(
        Path path, 
        Supplier<Pose2d> poseSupplier, 
        Consumer<ChassisSpeeds> robotRelativeSpeedsConsumer,
        PIDController translationController, 
        PIDController rotationController
    ) {
        if (translationController == null || rotationController == null) {
            throw new IllegalArgumentException("Translation and rotation controllers must be provided and must not be null");
        }

        this.path = path.copy();
        this.poseSupplier = poseSupplier;
        this.robotRelativeSpeedsConsumer = robotRelativeSpeedsConsumer;
        setTranslationController(translationController);
        setRotationController(rotationController);

        this.pathSegments = path.getPathSegments();
    }

    public FollowPath(
        Path path, 
        Supplier<Pose2d> poseSupplier, 
        Consumer<ChassisSpeeds> robotRelativeSpeedsConsumer
    ) {
        this(path, poseSupplier, robotRelativeSpeedsConsumer, translationController, rotationController);
    }

    @Override
    public void initialize() {
        if (translationController == null || rotationController == null) {
            throw new IllegalArgumentException("Translation and rotation controllers must be provided and must not be null");
        }
        this.segmentIndex = 0;
        this.rotationElementIndex = 0;
        this.lastTimestamp = Timer.getTimestamp();
    }

    private double lastTimestamp = 0;
    @Override
    public void periodic() {
        double dt = Timer.getTimestamp() - lastTimestamp;
        lastTimestamp = Timer.getTimestamp();

        Pose2d currentPose = poseSupplier.get();
    
       

       ChassisSpeeds targetSpeeds = new ChassisSpeeds();
       targetSpeeds = ChassisRateLimiter.limit(targetSpeeds, lastSpeeds, dt);

       robotRelativeSpeedsConsumer.accept(new ChassisSpeeds(vx, vy, omega));
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = poseSupplier.get();
        return 
            segmentIndex == pathSegments.size() - 1 && 
            rotationElementIndex == pathSegments.get(segmentIndex).size() - 2 &&
            currentPose.getTranslation().getDistance(
                ((TranslationTarget) pathSegments.get(segmentIndex).get(pathSegments.get(segmentIndex).size() -1)).translation()
            ) <= path.getEndTranslationToleranceMeters() &&
            currentPose.getRotation().minus(
                ((RotationTarget) pathSegments.get(segmentIndex).get(rotationElementIndex)).rotation()
            ).getDegrees() <= path.getEndRotationToleranceDeg();
    }
}
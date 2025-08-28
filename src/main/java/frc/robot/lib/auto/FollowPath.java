package frc.robot.lib.auto;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.auto.Path.PathElement;
import frc.robot.lib.auto.Path.PathElementConstraint;
import frc.robot.lib.auto.Path.RotationTarget;
import frc.robot.lib.auto.Path.RotationTargetConstraint;
import frc.robot.lib.auto.Path.TranslationTarget;
import frc.robot.lib.auto.Path.TranslationTargetConstraint;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

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
    private final Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier;
    private final Consumer<ChassisSpeeds> robotRelativeSpeedsConsumer;
    private final Supplier<Boolean> shouldFlipPathSupplier;
    private final Consumer<Pose2d> poseResetConsumer;

    private int rotationElementIndex = 0;
    private int translationElementIndex = 0;
    private ChassisSpeeds lastSpeeds = new ChassisSpeeds();
    private double lastTimestamp = 0;
    private Pose2d startPose = new Pose2d();
    private double lastRotationElementTargetRad = 0;   
    private int lastTranslationElementIndex = 0;
    private int lastRotationElementIndex = 0;
    private List<Pair<PathElement, PathElementConstraint>> pathElementsWithConstraints = new ArrayList<>();

    private int logCounter = 0;
    private ArrayList<Translation2d> robotTranslations = new ArrayList<>();

    public FollowPath(
        Path path, 
        SubsystemBase driveSubsystem, 
        Supplier<Pose2d> poseSupplier, 
        Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
        Consumer<ChassisSpeeds> robotRelativeSpeedsConsumer,
        Supplier<Boolean> shouldFlipPathSupplier,
        Consumer<Pose2d> poseResetConsumer,
        PIDController translationController, 
        PIDController rotationController
    ) {
        if (translationController == null || rotationController == null) {
            throw new IllegalArgumentException("Translation and rotation controllers must be provided and must not be null or must be set before calling FollowPath");
        }

        this.path = path.copy();
        this.poseSupplier = poseSupplier;
        this.robotRelativeSpeedsSupplier = robotRelativeSpeedsSupplier;
        this.robotRelativeSpeedsConsumer = robotRelativeSpeedsConsumer;
        this.shouldFlipPathSupplier = shouldFlipPathSupplier;
        this.poseResetConsumer = poseResetConsumer;
        setTranslationController(translationController);
        setRotationController(rotationController);
        
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(driveSubsystem);
    }

    public FollowPath(
        Path path, 
        SubsystemBase driveSubsystem, 
        Supplier<Pose2d> poseSupplier, 
        Consumer<Pose2d> poseResetConsumer,
        Supplier<Boolean> shouldFlipPathSupplier,
        Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
        Consumer<ChassisSpeeds> robotRelativeSpeedsConsumer
    ) {
        this(path, driveSubsystem, poseSupplier, robotRelativeSpeedsSupplier, robotRelativeSpeedsConsumer, shouldFlipPathSupplier, poseResetConsumer, translationController, rotationController);
    }

    @Override
    public void initialize() {
        if (translationController == null || rotationController == null) {
            throw new IllegalArgumentException("Translation and rotation controllers must be provided and must not be null");
        }

        if (shouldFlipPathSupplier.get()) {
            path.flip();
        }
        pathElementsWithConstraints = path.getPathElementsWithConstraintsNoWaypoints();

        rotationElementIndex = 0;
        translationElementIndex = 0;
        lastTimestamp = Timer.getTimestamp();
        lastSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeedsSupplier.get(), startPose.getRotation());
        startPose = poseSupplier.get();
        lastRotationElementTargetRad = startPose.getRotation().getRadians();
        lastTranslationElementIndex = translationElementIndex;
        lastRotationElementIndex = rotationElementIndex;
        rotationController.reset();
        translationController.reset();

        ArrayList<Translation2d> pathTranslations = new ArrayList<>();
        robotTranslations.clear();
        logCounter = 0;
        for (int i = 0; i < pathElementsWithConstraints.size(); i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                pathTranslations.add(((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation());
            }
        }
        Logger.recordOutput("FollowPath/pathTranslations", pathTranslations.toArray(Translation2d[]::new));

        // find the reset start pose. find the first translation target and use its translation as the start translation and the first rotation target as the start rotation. 
        // if no rotation target, use the current robot rotation as the start rotation
        Translation2d resetTranslation = ((TranslationTarget) pathElementsWithConstraints.get(0).getFirst()).translation();
        Rotation2d resetRotation = startPose.getRotation();
        for (int i = 0; i < pathElementsWithConstraints.size(); i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof RotationTarget) {
                resetRotation = ((RotationTarget) pathElementsWithConstraints.get(i).getFirst()).rotation();
                break;
            }
        }
        poseResetConsumer.accept(new Pose2d(resetTranslation, resetRotation));
        
    }

    @Override
    public void execute() {
        double dt = Timer.getTimestamp() - lastTimestamp;
        lastTimestamp = Timer.getTimestamp();

        Pose2d currentPose = poseSupplier.get();
        
        TranslationTarget currentTranslationTarget = (TranslationTarget) pathElementsWithConstraints.get(translationElementIndex).getFirst();
        // check to see if we are in the intermediate handoff radius of the current target translation
        if (currentPose.getTranslation().getDistance(currentTranslationTarget.translation()) <= 
            currentTranslationTarget.intermediateHandoffRadiusMeters().orElse(path.getDefaultGlobalConstraints().getIntermediateHandoffRadiusMeters())) {
            // if we are in the intermediate handoff radius of the current target translation,
            // switch to the next translation element
            for (int i = translationElementIndex + 1; i < pathElementsWithConstraints.size(); i++) {
                if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                    lastTranslationElementIndex = translationElementIndex;
                    translationElementIndex = i;
                    break;
                }
            }
            currentTranslationTarget = (TranslationTarget) pathElementsWithConstraints.get(translationElementIndex).getFirst();
        }

        // if we are not in the same segment as the current rotation element and we are ahead of the current rotation element in terms of segments, switch to the next rotation element until we are in the same segment
        // if we are in the same segment as the current rotation element, check to see if we have surpassed the current target rotation t_ratio (progress) and if so, 
        // switch to the next rotation element until the t_ratio of the rotation element is greater than the robot t_ratio or until the next rotation element is not in the same segment as the robot
        
        // check if the rotation target is in the previous segment by seeing if there are any translation targets between the current translation target index and the rotation target index
        while (rotationElementIndex < pathElementsWithConstraints.size() -  2 && 
            !((pathElementsWithConstraints.get(rotationElementIndex).getFirst() instanceof RotationTarget) &&
            isRotationTRatioGreater())) {

            Logger.recordOutput("FollowPath/rotationElementIndex", rotationElementIndex);
            rotationElementIndex++;
        }
        if (lastRotationElementIndex != rotationElementIndex && pathElementsWithConstraints.get(lastRotationElementIndex).getFirst() instanceof RotationTarget) {
            lastRotationElementTargetRad = ((RotationTarget) pathElementsWithConstraints.get(lastRotationElementIndex).getFirst()).rotation().getRadians();
            lastRotationElementIndex = rotationElementIndex;
        }

        Translation2d targetTranslation = ((TranslationTarget) pathElementsWithConstraints.get(translationElementIndex).getFirst()).translation();
        double remainingDistance = calculateRemainingPathDistance();
        double angleToTarget = Math.atan2(
            targetTranslation.getY() - currentPose.getTranslation().getY(), 
            targetTranslation.getX() - currentPose.getTranslation().getX()
        );
        double translationControllerOutput =  -translationController.calculate(remainingDistance, 0);
        double vx = translationControllerOutput * Math.cos(angleToTarget);
        double vy = translationControllerOutput * Math.sin(angleToTarget);

        RotationTarget currentRotationTarget = (RotationTarget) pathElementsWithConstraints.get(rotationElementIndex).getFirst();
        double targetRotation;
        if (currentRotationTarget.profiledRotation()) {
            double remainingRotationDistance = calculateRemainingDistanceToRotationTarget();
            double rotationSegmentDistance = calculateRotationTargetSegmentDistance();
            double segmentProgress = 1 - remainingRotationDistance / rotationSegmentDistance;
            targetRotation = lastRotationElementTargetRad + segmentProgress * (currentRotationTarget.rotation().getRadians() - lastRotationElementTargetRad);
        } else {
            targetRotation = currentRotationTarget.rotation().getRadians();
        }
        double omega = rotationController.calculate(currentPose.getRotation().getRadians(), targetRotation);

        TranslationTargetConstraint translationConstraint = (TranslationTargetConstraint) pathElementsWithConstraints.get(translationElementIndex).getSecond();
        RotationTargetConstraint rotationConstraint = (RotationTargetConstraint) pathElementsWithConstraints.get(rotationElementIndex).getSecond();

        ChassisSpeeds targetSpeeds = new ChassisSpeeds(vx, vy, omega);
        targetSpeeds = ChassisRateLimiter.limit(
            targetSpeeds, 
            lastSpeeds, 
            dt, 
            translationConstraint.maxAccelerationMetersPerSec2(),
            Math.toRadians(rotationConstraint.maxAccelerationDegPerSec2()),
            translationConstraint.maxVelocityMetersPerSec(),
            Math.toRadians(rotationConstraint.maxVelocityDegPerSec())
        );
        lastSpeeds = targetSpeeds;
        lastRotationElementTargetRad = currentPose.getRotation().getRadians();

        robotRelativeSpeedsConsumer.accept(ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, currentPose.getRotation()));

        if (logCounter++ % 3 == 0) {
            robotTranslations.add(currentPose.getTranslation());
            Logger.recordOutput("FollowPath/robotTranslations", robotTranslations.toArray(Translation2d[]::new));
        }

        Logger.recordOutput("FollowPath/calculateRemainingPathDistance", calculateRemainingPathDistance());
        Logger.recordOutput("FollowPath/straightLineDistToEnd", 
            currentPose.getTranslation().getDistance(
                ((TranslationTarget) pathElementsWithConstraints.get(pathElementsWithConstraints.size() - 1).getFirst()).translation()
            ));
        Logger.recordOutput("FollowPath/translationElementIndex", translationElementIndex);
        Logger.recordOutput("FollowPath/rotationElementIndex", rotationElementIndex);
        Logger.recordOutput("FollowPath/targetRotationElement", currentRotationTarget);
        Logger.recordOutput("FollowPath/targetRotation", targetRotation);

    }
    
    // initial test confirmed work
    private double calculateRemainingPathDistance() {
        Translation2d previousTranslation = poseSupplier.get().getTranslation();
        double remainingDistance = 0;
        for (int i = translationElementIndex; i < pathElementsWithConstraints.size(); i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                remainingDistance += previousTranslation.getDistance(
                    ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation()
                );
                previousTranslation = ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation();
            }
        }
        return remainingDistance;
    }

    private double calculateRemainingDistanceToRotationTarget() {
        Translation2d previousTranslation = poseSupplier.get().getTranslation();
        double remainingDistance = 0;
        if (isRotationNextSegment()) {
            for (int i = translationElementIndex; i <= rotationElementIndex; i++) {
                if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                    remainingDistance += previousTranslation.getDistance(
                        ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation()
                    );
                    previousTranslation = ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation();
                } else if (pathElementsWithConstraints.get(i).getFirst() instanceof RotationTarget) {
                    remainingDistance += previousTranslation.getDistance(
                        calculateRotationTargetTranslation(i)
                    );
                    previousTranslation = calculateRotationTargetTranslation(i);
                }
            }
        } else {
            remainingDistance = previousTranslation.getDistance(
                calculateRotationTargetTranslation(rotationElementIndex)
            );
        }
        
        return remainingDistance;
    }

    // total distance between the target rotation and the previous rotation target
    // if there is no previous rotation target, return the distance between the target rotation and the start of the path
    private double calculateRotationTargetSegmentDistance() {
        // Find the previous rotation target (immediately before current rotation)

        Translation2d startPoint = lastRotationElementIndex == 0
            ? startPose.getTranslation()
            : calculateRotationTargetTranslation(lastRotationElementIndex);
        Translation2d endPoint = calculateRotationTargetTranslation(rotationElementIndex);

        double distance = 0.0;
        Translation2d prev = startPoint;
        int startIdx = lastRotationElementIndex == 0 ? 0 : lastRotationElementIndex + 1;
        for (int i = startIdx; i <= rotationElementIndex; i++) {
            if (i == rotationElementIndex) {
                distance += prev.getDistance(endPoint);
                break;
            }
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                Translation2d next = ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation();
                distance += prev.getDistance(next);
                prev = next;
            }
        }
        return distance;
    }

    private Translation2d calculateRotationTargetTranslation(int index) {
        // find the two encompassing translation targets
        Translation2d translationA = null, translationB = null;
        for (int i = index; i >= 0; i--) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                translationA = ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation();
                break;
            }
        }
        for (int i = index; i < pathElementsWithConstraints.size(); i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                translationB = ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation();
                break;
            }
        }

        double angle = Math.atan2(
            translationB.getY() - translationA.getY(), 
            translationB.getX() - translationA.getX()
        );

        double tRatio = ((RotationTarget) pathElementsWithConstraints.get(index).getFirst()).t_ratio();
        Translation2d pointOnSegment = new Translation2d(
            translationA.getX() + Math.cos(angle) * tRatio,
            translationA.getY() + Math.sin(angle) * tRatio
        );

        Logger.recordOutput("FollowPath/calculateRotationTargetTranslation", new Pose2d(pointOnSegment, new Rotation2d()));
        return pointOnSegment;
    }


    private boolean isRotationTRatioGreater() {
        if (isRotationNextSegment()) { return true; }
        if (isRotationPreviousSegment()) { return false; }
        if (!(pathElementsWithConstraints.get(rotationElementIndex).getFirst() instanceof RotationTarget)) { return false; }

        Pose2d currentPose = poseSupplier.get();

        
        Translation2d translationA = translationElementIndex > 0 ? ((TranslationTarget) pathElementsWithConstraints.get(lastTranslationElementIndex).getFirst()).translation() : startPose.getTranslation();
        Translation2d translationB = ((TranslationTarget) pathElementsWithConstraints.get(translationElementIndex).getFirst()).translation();

        double segmentLength = translationA.getDistance(translationB);
        double segmentProgress = currentPose.getTranslation().getDistance(translationB) / segmentLength;
        boolean out = segmentProgress < ((RotationTarget) pathElementsWithConstraints.get(rotationElementIndex).getFirst()).t_ratio();

        Logger.recordOutput("FollowPath/isRotationTRatioGreater", out);
        return out;
    }
    
    private boolean isRotationPreviousSegment() {
        if (rotationElementIndex > translationElementIndex) { return false; }
        
        for (int i = rotationElementIndex; i < translationElementIndex; i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                return true;
            }
        }
        return false;
    }
    private boolean isRotationNextSegment() {
        return rotationElementIndex > translationElementIndex;
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = poseSupplier.get();
        // check if this is the last rotation element
        boolean isLastRotationElement = true;
        for (int i = rotationElementIndex+1; i < pathElementsWithConstraints.size(); i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof RotationTarget) {
                isLastRotationElement = false;
                break;
            }
        }
        boolean isLastTranslationElement = translationElementIndex == pathElementsWithConstraints.size() - 1;
        boolean finished = isLastRotationElement && isLastTranslationElement && 
            currentPose.getTranslation().getDistance(
                ((TranslationTarget) pathElementsWithConstraints.get(translationElementIndex).getFirst()).translation()
            ) <= path.getEndTranslationToleranceMeters() &&
            currentPose.getRotation().minus(
                ((RotationTarget) pathElementsWithConstraints.get(rotationElementIndex).getFirst()).rotation()
            ).getDegrees() <= path.getEndRotationToleranceDeg();

        Logger.recordOutput("FollowPath/finished", finished);
        return finished;
    }
}
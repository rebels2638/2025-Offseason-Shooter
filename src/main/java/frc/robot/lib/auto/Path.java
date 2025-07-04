package frc.robot.lib.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Path {
    public sealed interface PathElement permits Waypoint, TranslationTarget, RotationTarget {}
    public static record Waypoint(
        TranslationTarget translationTarget,
        RotationTarget rotationTarget
    ) implements PathElement {}

    public static record TranslationTarget(
        Translation2d translation, 
        Optional<Double> finalVelocityMetersPerSec, 
        Optional<Double> maxVelocityMetersPerSec, 
        Optional<Double> maxAccelerationMetersSPerSec2,
        Optional<Double> intermediateHandoffRadiusMeters
    ) implements PathElement {}

    public static record RotationTarget(
        Rotation2d rotation, 
        Translation2d translation, 
        Optional<Double> maxVelocityRadPerSec, 
        Optional<Double> maxAccelerationRadSPerSec2
    ) implements PathElement {}

    private List<PathElement> pathElements = new ArrayList<PathElement>();
    
    public Path(List<PathElement> pathElements) {
        this.pathElements = pathElements;
    }

    public Path() {}

    public Path addPathElement(PathElement pathElement) {
        pathElements.add(pathElement);
        return this;
    }

    public Path addWaypoint(
        TranslationTarget translationTarget, 
        RotationTarget rotationTarget
    ) {
        pathElements.add(new Waypoint(translationTarget, rotationTarget));
        return this;
    }

    public Path addWaypoint(
        Translation2d translation, 
        Optional<Double> finalVelocityMetersPerSec, 
        Optional<Double> maxVelocityMetersPerSec, 
        Optional<Double> maxAccelerationMetersSPerSec2,
        Optional<Double> intermediateHandoffRadiusMeters,
        Rotation2d rotation, 
        Optional<Double> maxVelocityRadPerSec, 
        Optional<Double> maxAccelerationRadSPerSec2
    ) {
        return addWaypoint(
            new TranslationTarget(translation, finalVelocityMetersPerSec, maxVelocityMetersPerSec, maxAccelerationMetersSPerSec2, intermediateHandoffRadiusMeters),
            new RotationTarget(rotation, translation, maxVelocityRadPerSec, maxAccelerationRadSPerSec2)
        );
    }

    public Path addTranslationTarget(
        Translation2d translation, 
        Optional<Double> finalVelocityMetersPerSec, 
        Optional<Double> maxVelocityMetersPerSec, 
        Optional<Double> maxAccelerationMetersSPerSec2,
        Optional<Double> intermediateHandoffRadiusMeters
    ) {
        pathElements.add(new TranslationTarget(translation, finalVelocityMetersPerSec, maxVelocityMetersPerSec, maxAccelerationMetersSPerSec2, intermediateHandoffRadiusMeters));
        return this;
    }

    public Path addRotationTarget(
        Rotation2d rotation, 
        Translation2d translation, 
        Optional<Double> maxVelocityRadPerSec, 
        Optional<Double> maxAccelerationRadSPerSec2
    ) {
        pathElements.add(new RotationTarget(rotation, translation, maxVelocityRadPerSec, maxAccelerationRadSPerSec2));
        return this;
    }

    public List<PathElement> getPathElements() {
        return pathElements;
    }

    public void setPathElements(List<PathElement> pathElements) {
        this.pathElements = pathElements;
    }

    public List<Translation2d> getTranslationTargets() {
        List<Translation2d> translations = new ArrayList<>();
        for (PathElement element : pathElements) {
            if (element instanceof TranslationTarget translationTarget) {
                translations.add(translationTarget.translation);
            } else if (element instanceof Waypoint waypoint) {
                translations.add(waypoint.translationTarget.translation);
            }
        }
        return translations;
    }

    public List<Rotation2d> getRotationTargets() {
        List<Rotation2d> rotations = new ArrayList<>();
        for (PathElement element : pathElements) {
            if (element instanceof RotationTarget rotationTarget) {
                rotations.add(rotationTarget.rotation);
            } else if (element instanceof Waypoint waypoint) {
                rotations.add(waypoint.rotationTarget.rotation);
            }
        }
        return rotations;
    }

    public List<TranslationTarget> getTranslationPathElements() {
        List<TranslationTarget> translationTargets = new ArrayList<>();
        for (PathElement element : pathElements) {
            if (element instanceof TranslationTarget translationTarget) {
                translationTargets.add(translationTarget);
            } else if (element instanceof Waypoint waypoint) {
                translationTargets.add(waypoint.translationTarget);
            }
        }
        return translationTargets;
    }

    public List<RotationTarget> getRotationPathElements() {
        List<RotationTarget> rotationTargets = new ArrayList<>();
        for (PathElement element : pathElements) {
            if (element instanceof RotationTarget rotationTarget) {
                rotationTargets.add(rotationTarget);
            } else if (element instanceof Waypoint waypoint) {
                rotationTargets.add(waypoint.rotationTarget);
            }
        }
        return rotationTargets;
    }

    public List<Boolean> getTranslationTargetWaypointOverlap() {
        List<Boolean> overlap = new ArrayList<>();
        for (PathElement element : pathElements) {
            overlap.add(element instanceof Waypoint || element instanceof TranslationTarget);
        }
        return overlap;
    } 

    public int indexOf(PathElement element) {
        for (int i = 0; i < pathElements.size(); i++) {
            if (pathElements.get(i).equals(element)) {
                return i;
            }
        }
        return -1; // Element not found
    }
    
    public int indexOfRotationTarget(RotationTarget rt) {
        for (int i = 0; i < pathElements.size(); i++) {
            PathElement e = pathElements.get(i);
            if (e instanceof RotationTarget r && r.equals(rt)) return i;
            if (e instanceof Waypoint w && w.rotationTarget().equals(rt)) return i;
        }
        return -1;
    }

    public int indexOfTranslationTarget(TranslationTarget tt) {
        for (int i = 0; i < pathElements.size(); i++) {
            PathElement e = pathElements.get(i);
            if (e instanceof TranslationTarget t && t.equals(tt)) return i;
            if (e instanceof Waypoint w && w.translationTarget().equals(tt)) return i;
        }
        return -1;
    }

    public List<Translation2d> getAllPathElementTranslations() {
        List<Translation2d> translations = new ArrayList<>();
        for (PathElement element : pathElements) {
            if (element instanceof TranslationTarget translationTarget) {
                translations.add(translationTarget.translation);
            } else if (element instanceof Waypoint waypoint) {
                translations.add(waypoint.translationTarget.translation);
            } else if (element instanceof RotationTarget rotationTarget) {
                translations.add(rotationTarget.translation);
            }
        }
        return translations;
    }
}

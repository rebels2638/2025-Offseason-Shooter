package frc.robot.lib.auto;

import java.io.File;
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
        Optional<Double> intermediateHandoffRadiusMeters
    ) implements PathElement {}

    public static record RotationTarget(
        Rotation2d rotation, 
        double t_ratio,
        boolean profiledRotation
    ) implements PathElement {}

    // New constraint model to mirror the provided Python structure
    public static record RangedConstraint(
        double value,
        int startOrdinal,
        int endOrdinal
    ) {}

    public static final class DefaultGlobalConstraints {
        private final double maxVelocityMetersPerSec;
        private final double maxAccelerationMetersPerSec2;
        private final double maxVelocityDegPerSec;
        private final double maxAccelerationDegPerSec2;
        private final double endTranslationToleranceMeters;
        private final double endRotationToleranceDeg;
        private final double intermediateHandoffRadiusMeters;

        public DefaultGlobalConstraints(
            double maxVelocityMetersPerSec,
            double maxAccelerationMetersPerSec2,
            double maxVelocityDegPerSec,
            double maxAccelerationDegPerSec2,
            double endTranslationToleranceMeters,
            double endRotationToleranceDeg,
            double intermediateHandoffRadiusMeters
        ) {
            this.maxVelocityMetersPerSec = maxVelocityMetersPerSec;
            this.maxAccelerationMetersPerSec2 = maxAccelerationMetersPerSec2;
            this.maxVelocityDegPerSec = maxVelocityDegPerSec;
            this.maxAccelerationDegPerSec2 = maxAccelerationDegPerSec2;
            this.endTranslationToleranceMeters = endTranslationToleranceMeters;
            this.endRotationToleranceDeg = endRotationToleranceDeg;
            this.intermediateHandoffRadiusMeters = intermediateHandoffRadiusMeters;
        }

        public double getMaxVelocityMetersPerSec() { return maxVelocityMetersPerSec; }
        public double getMaxAccelerationMetersPerSec2() { return maxAccelerationMetersPerSec2; }
        public double getMaxVelocityDegPerSec() { return maxVelocityDegPerSec; }
        public double getMaxAccelerationDegPerSec2() { return maxAccelerationDegPerSec2; }
        public double getEndTranslationToleranceMeters() { return endTranslationToleranceMeters; }
        public double getEndRotationToleranceDeg() { return endRotationToleranceDeg; }
        public double getIntermediateHandoffRadiusMeters() { return intermediateHandoffRadiusMeters; }
    }
    
    public static final class PathConstraints {
        private Optional<RangedConstraint[]> maxVelocityMetersPerSec = Optional.empty();
        private Optional<RangedConstraint[]> maxAccelerationMetersPerSec2 = Optional.empty();
        private Optional<RangedConstraint[]> maxVelocityDegPerSec = Optional.empty();
        private Optional<RangedConstraint[]> maxAccelerationDegPerSec2 = Optional.empty();
        private Optional<Double> endTranslationToleranceMeters = Optional.empty();
        private Optional<Double> endRotationToleranceDeg = Optional.empty();

        public PathConstraints() {}

        public Optional<RangedConstraint[]> getMaxVelocityMetersPerSec() { return maxVelocityMetersPerSec.map(arr -> arr.clone()); }
        public void setMaxVelocityMetersPerSec(Optional<RangedConstraint[]> v) { this.maxVelocityMetersPerSec = v.map(arr -> arr.clone()); }

        public Optional<RangedConstraint[]> getMaxAccelerationMetersPerSec2() { return maxAccelerationMetersPerSec2.map(arr -> arr.clone()); }
        public void setMaxAccelerationMetersPerSec2(Optional<RangedConstraint[]> v) { this.maxAccelerationMetersPerSec2 = v.map(arr -> arr.clone()); }

        public Optional<RangedConstraint[]> getMaxVelocityDegPerSec() { return maxVelocityDegPerSec.map(arr -> arr.clone()); }
        public void setMaxVelocityDegPerSec(Optional<RangedConstraint[]> v) { this.maxVelocityDegPerSec = v.map(arr -> arr.clone()); }

        public Optional<RangedConstraint[]> getMaxAccelerationDegPerSec2() { return maxAccelerationDegPerSec2.map(arr -> arr.clone()); }
        public void setMaxAccelerationDegPerSec2(Optional<RangedConstraint[]> v) { this.maxAccelerationDegPerSec2 = v.map(arr -> arr.clone()); }

        public Optional<Double> getEndTranslationToleranceMeters() { return endTranslationToleranceMeters; }
        public void setEndTranslationToleranceMeters(Optional<Double> v) { this.endTranslationToleranceMeters = v; }

        public Optional<Double> getEndRotationToleranceDeg() { return endRotationToleranceDeg; }
        public void setEndRotationToleranceDeg(Optional<Double> v) { this.endRotationToleranceDeg = v; }

        public PathConstraints copy() {
            PathConstraints c = new PathConstraints();
            c.setMaxVelocityMetersPerSec(getMaxVelocityMetersPerSec());
            c.setMaxAccelerationMetersPerSec2(getMaxAccelerationMetersPerSec2());
            c.setMaxVelocityDegPerSec(getMaxVelocityDegPerSec());
            c.setMaxAccelerationDegPerSec2(getMaxAccelerationDegPerSec2());
            c.setEndTranslationToleranceMeters(getEndTranslationToleranceMeters());
            c.setEndRotationToleranceDeg(getEndRotationToleranceDeg());
            return c;
        }
    }


    private List<PathElement> pathElements;
    private PathConstraints constraints;
    private DefaultGlobalConstraints globalConstraints;
    
    public Path(List<PathElement> pathElements, PathConstraints constraints, DefaultGlobalConstraints globalConstraints) {
        this.pathElements = pathElements == null ? new ArrayList<>() : new ArrayList<>(pathElements);
        this.constraints = constraints == null ? new PathConstraints() : constraints.copy();
        this.globalConstraints = globalConstraints == null ? new DefaultGlobalConstraints(0,0,0,0,0,0,0) : new DefaultGlobalConstraints(
            globalConstraints.getMaxVelocityMetersPerSec(),
            globalConstraints.getMaxAccelerationMetersPerSec2(),
            globalConstraints.getMaxVelocityDegPerSec(),
            globalConstraints.getMaxAccelerationDegPerSec2(),
            globalConstraints.getEndTranslationToleranceMeters(),
            globalConstraints.getEndRotationToleranceDeg(),
            globalConstraints.getIntermediateHandoffRadiusMeters()
        );
    }

    public Path(File autosDir, String pathFileName) {
        Path loaded = JsonUtils.loadPath(autosDir, pathFileName);
        this.pathElements = new ArrayList<>(loaded.pathElements);
        this.constraints = loaded.constraints.copy();
        this.globalConstraints = new DefaultGlobalConstraints(
            loaded.globalConstraints.getMaxVelocityMetersPerSec(),
            loaded.globalConstraints.getMaxAccelerationMetersPerSec2(),
            loaded.globalConstraints.getMaxVelocityDegPerSec(),
            loaded.globalConstraints.getMaxAccelerationDegPerSec2(),
            loaded.globalConstraints.getEndTranslationToleranceMeters(),
            loaded.globalConstraints.getEndRotationToleranceDeg(),
            loaded.globalConstraints.getIntermediateHandoffRadiusMeters()
        );
    }

    public Path(String pathFileName) {
        this(new File(JsonUtils.PROJECT_ROOT), pathFileName);
    }

    public Path() {
        this(new ArrayList<>(), new PathConstraints(), new DefaultGlobalConstraints(0,0,0,0,0,0,0));
    }

    public DefaultGlobalConstraints getDefaultGlobalConstraints() {
        return new DefaultGlobalConstraints(
            globalConstraints.getMaxVelocityMetersPerSec(),
            globalConstraints.getMaxAccelerationMetersPerSec2(),
            globalConstraints.getMaxVelocityDegPerSec(),
            globalConstraints.getMaxAccelerationDegPerSec2(),
            globalConstraints.getEndTranslationToleranceMeters(),
            globalConstraints.getEndRotationToleranceDeg(),
            globalConstraints.getIntermediateHandoffRadiusMeters()
        );
    }

    public PathConstraints getPathConstraints() { return constraints.copy(); }
    public void setPathConstraints(PathConstraints pathConstraints) { this.constraints = pathConstraints == null ? new PathConstraints() : pathConstraints.copy(); }

    public Path addPathElement(PathElement pathElement) {
        pathElements.add(pathElement);
        return this;
    }

    public PathElement getElement(int index) {
        if (index >= 0 && index < pathElements.size()) {
            return pathElements.get(index);
        }
        throw new IndexOutOfBoundsException("Index out of range");
    }

    public void setElement(int index, PathElement element) {
        if (index >= 0 && index < pathElements.size()) {
            pathElements.set(index, element);
            return;
        }
        throw new IndexOutOfBoundsException("Index out of range");
    }

    public PathElement removeElement(int index) {
        if (index >= 0 && index < pathElements.size()) {
            return pathElements.remove(index);
        }
        throw new IndexOutOfBoundsException("Index out of range");
    }

    public Path reorderElements(List<Integer> newOrder) {
        if (newOrder.size() != pathElements.size()) {
            throw new IllegalArgumentException("New order must match elements length");
        }
        List<PathElement> reordered = new ArrayList<>(pathElements.size());
        for (int i : newOrder) {
            reordered.add(pathElements.get(i));
        }
        this.pathElements = reordered;
        return this;
    }
    public List<PathElement> getPathElements() { return new ArrayList<>(pathElements); }

    public void setPathElements(List<PathElement> pathElements) { this.pathElements = pathElements == null ? new ArrayList<>() : new ArrayList<>(pathElements); }

    // Removed redundant getConstraints/setConstraints to avoid confusion; use getPathConstraints/setPathConstraints instead.
}   
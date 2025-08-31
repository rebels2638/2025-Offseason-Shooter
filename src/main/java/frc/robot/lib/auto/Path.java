package frc.robot.lib.auto;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.logging.Level;
import java.util.logging.Logger;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Path {
    private static final Logger logger = Logger.getLogger(Path.class.getName());

    public sealed interface PathElement permits Waypoint, TranslationTarget, RotationTarget {
        public PathElement copy();
    }
    public sealed interface PathElementConstraint permits WaypointConstraint, TranslationTargetConstraint, RotationTargetConstraint {}
    public static record WaypointConstraint(
        double maxVelocityMetersPerSec,
        double maxAccelerationMetersPerSec2,
        double maxVelocityDegPerSec,
        double maxAccelerationDegPerSec2
    ) implements PathElementConstraint {}
    public static record TranslationTargetConstraint(
        double maxVelocityMetersPerSec,
        double maxAccelerationMetersPerSec2
    ) implements PathElementConstraint {}
    public static record RotationTargetConstraint(
        double maxVelocityDegPerSec,
        double maxAccelerationDegPerSec2
    ) implements PathElementConstraint {}

    public static record Waypoint(
        TranslationTarget translationTarget,
        RotationTarget rotationTarget
    ) implements PathElement {
        public Waypoint copy() {
            return new Waypoint(translationTarget.copy(), rotationTarget.copy());
        }
    }

    public static record TranslationTarget(
        Translation2d translation, 
        Optional<Double> intermediateHandoffRadiusMeters
    ) implements PathElement {
        public TranslationTarget copy() {
            return new TranslationTarget(translation, intermediateHandoffRadiusMeters);
        }
    }

    public static record RotationTarget(
        Rotation2d rotation, 
        double t_ratio,
        boolean profiledRotation
    ) implements PathElement {
        public RotationTarget copy() {
            return new RotationTarget(rotation, t_ratio, profiledRotation);
        }
    }

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

        public DefaultGlobalConstraints copy() {
            return new DefaultGlobalConstraints(
                maxVelocityMetersPerSec,
                maxAccelerationMetersPerSec2,
                maxVelocityDegPerSec,
                maxAccelerationDegPerSec2,
                endTranslationToleranceMeters,
                endRotationToleranceDeg,
                intermediateHandoffRadiusMeters
            );
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
        private Optional<ArrayList<RangedConstraint>> maxVelocityMetersPerSec = Optional.empty();
        private Optional<ArrayList<RangedConstraint>> maxAccelerationMetersPerSec2 = Optional.empty();
        private Optional<ArrayList<RangedConstraint>> maxVelocityDegPerSec = Optional.empty();
        private Optional<ArrayList<RangedConstraint>> maxAccelerationDegPerSec2 = Optional.empty();
        private Optional<Double> endTranslationToleranceMeters = Optional.empty();
        private Optional<Double> endRotationToleranceDeg = Optional.empty();

        public PathConstraints() {}

        public Optional<ArrayList<RangedConstraint>> getMaxVelocityMetersPerSec() { return maxVelocityMetersPerSec.map(list -> new ArrayList<>(list)); }
        public void setMaxVelocityMetersPerSec(Optional<ArrayList<RangedConstraint>> v) { this.maxVelocityMetersPerSec = v.map(list -> new ArrayList<>(list)); }

        public Optional<ArrayList<RangedConstraint>> getMaxAccelerationMetersPerSec2() { return maxAccelerationMetersPerSec2.map(list -> new ArrayList<>(list)); }
        public void setMaxAccelerationMetersPerSec2(Optional<ArrayList<RangedConstraint>> v) { this.maxAccelerationMetersPerSec2 = v.map(list -> new ArrayList<>(list)); }

        public Optional<ArrayList<RangedConstraint>> getMaxVelocityDegPerSec() { return maxVelocityDegPerSec.map(list -> new ArrayList<>(list)); }
        public void setMaxVelocityDegPerSec(Optional<ArrayList<RangedConstraint>> v) { this.maxVelocityDegPerSec = v.map(list -> new ArrayList<>(list)); }

        public Optional<ArrayList<RangedConstraint>> getMaxAccelerationDegPerSec2() { return maxAccelerationDegPerSec2.map(list -> new ArrayList<>(list)); }
        public void setMaxAccelerationDegPerSec2(Optional<ArrayList<RangedConstraint>> v) { this.maxAccelerationDegPerSec2 = v.map(list -> new ArrayList<>(list)); }

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
    private PathConstraints pathConstraints;
    private static DefaultGlobalConstraints defaultGlobalConstraints = null;
    private boolean flipped = false;
    private boolean isValid = true;
    
    public Path(List<PathElement> pathElements, PathConstraints constraints, DefaultGlobalConstraints defaultGlobalConstraints) {
        if (pathElements == null) {
            throw new IllegalArgumentException("pathElements cannot be null");
        }
        if (constraints == null) {
            throw new IllegalArgumentException("constraints cannot be null");
        }
        if (defaultGlobalConstraints == null) {
            throw new IllegalArgumentException("defaultGlobalConstraints must be set before creating a path and defaultGlobalConstraints cannot be null");
        }
        
        this.pathElements = new ArrayList<>(pathElements);
        this.pathConstraints = constraints.copy();
        Path.defaultGlobalConstraints = defaultGlobalConstraints.copy();
        
        // Validate that first and last elements are both either waypoints or translation targets
        validatePathEndpoints();
    }

    public Path(List<PathElement> pathElements, PathConstraints constraints) {
        this(pathElements, constraints, Path.defaultGlobalConstraints);
    }

    public Path(File autosDir, String pathFileName) {
        Path loaded = JsonUtils.loadPath(autosDir, pathFileName+".json");
        this.pathElements = loaded.pathElements;
        this.pathConstraints = loaded.pathConstraints;
        // globals are static and already copied

        // Validate that first and last elements are both either waypoints or translation targets
        validatePathEndpoints();
    }

    public Path(String pathFileName) {
        this(new File(JsonUtils.PROJECT_ROOT), pathFileName);
    }

    /**
     * Validates that the first and last path elements are both either waypoints or translation targets.
     * If the path has only 1 element, it must be a waypoint or translation target.
     * Logs an error if validation fails but allows program execution to continue.
     */
    private void validatePathEndpoints() {
        if (pathElements.size() == 0) {
            isValid = false;
            logger.log(Level.WARNING, "Path validation failed: Path cannot be empty");
            return;
        }

        if (pathElements.size() == 1) {
            PathElement element = pathElements.get(0);
            if (element instanceof RotationTarget) {
                isValid = false;
                logger.log(Level.WARNING, "Path validation failed: Path cannot consist of a single rotation target");
            }
            return;
        }

        PathElement first = pathElements.get(0);
        PathElement last = pathElements.get(pathElements.size() - 1);

        boolean firstIsValid = first instanceof Waypoint || first instanceof TranslationTarget;
        boolean lastIsValid = last instanceof Waypoint || last instanceof TranslationTarget;

        if (!firstIsValid || !lastIsValid) {
            isValid = false;
            logger.log(Level.WARNING, "Path validation failed: First and last path elements must both be either Waypoints or TranslationTargets. " +
                "First element is: " + first.getClass().getSimpleName() + ", " +
                "Last element is: " + last.getClass().getSimpleName());
        }
    }

    public boolean isValid() {
        return isValid;
    }

    public DefaultGlobalConstraints getDefaultGlobalConstraints() {
        return defaultGlobalConstraints.copy();
    }
    public void setDefaultGlobalConstraints(DefaultGlobalConstraints defaultGlobalConstraints) {
        if (defaultGlobalConstraints == null) {
            throw new IllegalArgumentException("defaultGlobalConstraints cannot be null");
        }
        Path.defaultGlobalConstraints = defaultGlobalConstraints.copy();
    }

    public PathConstraints getPathConstraints() { return pathConstraints.copy(); }
    public void setPathConstraints(PathConstraints pathConstraints) { 
        if (pathConstraints == null) {
            throw new IllegalArgumentException("pathConstraints cannot be null");
        }
        this.pathConstraints = pathConstraints.copy(); 
    }

    public double getEndTranslationToleranceMeters() {
        return pathConstraints.getEndTranslationToleranceMeters().orElse(defaultGlobalConstraints.getEndTranslationToleranceMeters());
    }
    public double getEndRotationToleranceDeg() {
        return pathConstraints.getEndRotationToleranceDeg().orElse(defaultGlobalConstraints.getEndRotationToleranceDeg());
    }

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

    public void setPathElements(List<PathElement> pathElements) { 
        if (pathElements == null) {
            throw new IllegalArgumentException("pathElements cannot be null");
        }
        this.pathElements = new ArrayList<>(pathElements); 
    }

    public List<Pair<PathElement, PathElementConstraint>> getPathElementsWithConstraints() {
        if (!isValid()) {
            return new ArrayList<>();
        }
        
        List<Pair<PathElement, PathElementConstraint>> elementsWithConstraints = new ArrayList<>();
        int translationOrdinal = 0;
        int rotationOrdinal = 0;
        for (PathElement element : pathElements) {
            if (element instanceof Waypoint) {
                double maxVelocityMetersPerSec = -1;
                double maxAccelerationMetersPerSec2 = -1;
                double maxVelocityDegPerSec = -1;
                double maxAccelerationDegPerSec2 = -1;
                if (pathConstraints.getMaxVelocityMetersPerSec().isPresent()) {
                    for (RangedConstraint constraint : pathConstraints.getMaxVelocityMetersPerSec().get()) {
                        if (constraint.startOrdinal() <= translationOrdinal && constraint.endOrdinal() >= translationOrdinal) {
                            maxVelocityMetersPerSec = constraint.value();
                            break;
                        }
                    }
                }
                if (pathConstraints.getMaxAccelerationMetersPerSec2().isPresent()) {
                    for (RangedConstraint constraint : pathConstraints.getMaxAccelerationMetersPerSec2().get()) {
                        if (constraint.startOrdinal() <= translationOrdinal && constraint.endOrdinal() >= translationOrdinal) {
                            maxAccelerationMetersPerSec2 = constraint.value();
                            break;
                        }
                    }
                }
                if (pathConstraints.getMaxVelocityDegPerSec().isPresent()) {
                    for (RangedConstraint constraint : pathConstraints.getMaxVelocityDegPerSec().get()) {
                        if (constraint.startOrdinal() <= rotationOrdinal && constraint.endOrdinal() >= rotationOrdinal) {
                            maxVelocityDegPerSec = constraint.value();
                            break;
                        }
                    }
                }
                if (pathConstraints.getMaxAccelerationDegPerSec2().isPresent()) {
                    for (RangedConstraint constraint : pathConstraints.getMaxAccelerationDegPerSec2().get()) {
                        if (constraint.startOrdinal() <= rotationOrdinal && constraint.endOrdinal() >= rotationOrdinal) {
                            maxAccelerationDegPerSec2 = constraint.value();
                            break;
                        }
                    }
                }
                maxVelocityMetersPerSec = maxVelocityMetersPerSec == -1 ? 
                    defaultGlobalConstraints.getMaxVelocityMetersPerSec() : maxVelocityMetersPerSec;
                maxAccelerationMetersPerSec2 = maxAccelerationMetersPerSec2 == -1 ? 
                    defaultGlobalConstraints.getMaxAccelerationMetersPerSec2() : maxAccelerationMetersPerSec2;
                maxVelocityDegPerSec = maxVelocityDegPerSec == -1 ? 
                    defaultGlobalConstraints.getMaxVelocityDegPerSec() : maxVelocityDegPerSec;
                maxAccelerationDegPerSec2 = maxAccelerationDegPerSec2 == -1 ? 
                    defaultGlobalConstraints.getMaxAccelerationDegPerSec2() : maxAccelerationDegPerSec2;

                elementsWithConstraints.add(
                    new Pair<>(
                        element, 
                        new WaypointConstraint(
                            maxVelocityMetersPerSec, 
                            maxAccelerationMetersPerSec2, 
                            maxVelocityDegPerSec, 
                            maxAccelerationDegPerSec2
                        )
                    )
                );
                translationOrdinal++;
                rotationOrdinal++;
            }
            else if (element instanceof TranslationTarget) {
                double maxVelocityMetersPerSec = -1;
                double maxAccelerationMetersPerSec2 = -1;
                if (pathConstraints.getMaxVelocityMetersPerSec().isPresent()) {
                    for (RangedConstraint constraint : pathConstraints.getMaxVelocityMetersPerSec().get()) {
                        if (constraint.startOrdinal() <= translationOrdinal && constraint.endOrdinal() >= translationOrdinal) {
                            maxVelocityMetersPerSec = constraint.value();
                            break;
                        }
                    }
                }
                if (pathConstraints.getMaxAccelerationMetersPerSec2().isPresent()) {
                    for (RangedConstraint constraint : pathConstraints.getMaxAccelerationMetersPerSec2().get()) {
                        if (constraint.startOrdinal() <= translationOrdinal && constraint.endOrdinal() >= translationOrdinal) {
                            maxAccelerationMetersPerSec2 = constraint.value();
                            break;
                        }
                    }
                }
                maxVelocityMetersPerSec = maxVelocityMetersPerSec == -1 ? 
                    defaultGlobalConstraints.getMaxVelocityMetersPerSec() : maxVelocityMetersPerSec;
                maxAccelerationMetersPerSec2 = maxAccelerationMetersPerSec2 == -1 ? 
                    defaultGlobalConstraints.getMaxAccelerationMetersPerSec2() : maxAccelerationMetersPerSec2;

                elementsWithConstraints.add(
                    new Pair<>(
                        element, 
                        new TranslationTargetConstraint(
                            maxVelocityMetersPerSec, 
                            maxAccelerationMetersPerSec2
                        )
                    )
                );
                translationOrdinal++;
            }
            else if (element instanceof RotationTarget) {
                double maxVelocityDegPerSec = -1;
                double maxAccelerationDegPerSec2 = -1;
                if (pathConstraints.getMaxVelocityDegPerSec().isPresent()) {
                    for (RangedConstraint constraint : pathConstraints.getMaxVelocityDegPerSec().get()) {
                        if (constraint.startOrdinal() <= rotationOrdinal && constraint.endOrdinal() >= rotationOrdinal) {
                            maxVelocityDegPerSec = constraint.value();
                            break;
                        }
                    }
                }
                if (pathConstraints.getMaxAccelerationDegPerSec2().isPresent()) {
                    for (RangedConstraint constraint : pathConstraints.getMaxAccelerationDegPerSec2().get()) {
                        if (constraint.startOrdinal() <= rotationOrdinal && constraint.endOrdinal() >= rotationOrdinal) {
                            maxAccelerationDegPerSec2 = constraint.value();
                            break;
                        }
                    }
                }
                maxVelocityDegPerSec = maxVelocityDegPerSec == -1 ? 
                    defaultGlobalConstraints.getMaxVelocityDegPerSec() : maxVelocityDegPerSec;
                maxAccelerationDegPerSec2 = maxAccelerationDegPerSec2 == -1 ? 
                    defaultGlobalConstraints.getMaxAccelerationDegPerSec2() : maxAccelerationDegPerSec2;
                elementsWithConstraints.add(new Pair<>(element, new RotationTargetConstraint(maxVelocityDegPerSec, maxAccelerationDegPerSec2)));

                rotationOrdinal++;
            }
        }

        return elementsWithConstraints;
    }

    public List<Pair<PathElement, PathElementConstraint>> getPathElementsWithConstraintsNoWaypoints() {
        if (!isValid()) {
            return new ArrayList<>();
        }
        
        List<Pair<PathElement, PathElementConstraint>> elementsWithConstraints = getPathElementsWithConstraints();
        List<Pair<PathElement, PathElementConstraint>> out = new ArrayList<>();
        for (int i = 0; i < elementsWithConstraints.size(); i++) {
            PathElement element = elementsWithConstraints.get(i).getFirst();
            PathElementConstraint constraint = elementsWithConstraints.get(i).getSecond();
            if (element instanceof Waypoint) {
                TranslationTarget translationTarget = ((Waypoint)element).translationTarget();
                TranslationTargetConstraint translationTargetConstraint = new TranslationTargetConstraint(
                    ((WaypointConstraint)constraint).maxVelocityMetersPerSec(), 
                    ((WaypointConstraint)constraint).maxAccelerationMetersPerSec2()
                );
                RotationTarget rotationTarget = ((Waypoint)element).rotationTarget();
                RotationTargetConstraint rotationTargetConstraint = new RotationTargetConstraint(
                    ((WaypointConstraint)constraint).maxVelocityDegPerSec(), 
                    ((WaypointConstraint)constraint).maxAccelerationDegPerSec2()
                );
                if (i == 0) {
                    rotationTarget = new RotationTarget(
                        rotationTarget.rotation(), 
                        0, 
                        rotationTarget.profiledRotation()
                    );

                    out.add(new Pair<>(
                        translationTarget, 
                        translationTargetConstraint
                    ));
                    out.add(new Pair<>(
                        rotationTarget, 
                        rotationTargetConstraint
                    ));
                } else {
                    rotationTarget = new RotationTarget(
                        rotationTarget.rotation(), 
                        1, 
                        rotationTarget.profiledRotation()
                    );
                    out.add(new Pair<>(
                        rotationTarget, 
                        rotationTargetConstraint
                    ));
                    out.add(new Pair<>(
                        translationTarget, 
                        translationTargetConstraint
                    ));
                }
            } else {
                out.add(elementsWithConstraints.get(i));
            }
        }
        return out;
    }
    public void flip() {
        if (!isValid()) {
            return;
        }
        
        if (flipped) return;

        for (int i = 0; i < pathElements.size(); i++) {
            PathElement element = pathElements.get(i);
            if (element instanceof TranslationTarget) {
                pathElements.set(i, new TranslationTarget(
                    FlippingUtil.flipFieldPosition(((TranslationTarget) element).translation()),
                    ((TranslationTarget) element).intermediateHandoffRadiusMeters()
                ));
            } else if (element instanceof RotationTarget) {
                pathElements.set(i, new RotationTarget(
                    FlippingUtil.flipFieldRotation(((RotationTarget) element).rotation()),
                    ((RotationTarget) element).t_ratio(),
                    ((RotationTarget) element).profiledRotation()
                ));
            } else if (element instanceof Waypoint) {
                pathElements.set(i, new Waypoint(
                    new TranslationTarget(
                        FlippingUtil.flipFieldPosition(((Waypoint) element).translationTarget().translation()),
                        ((Waypoint) element).translationTarget().intermediateHandoffRadiusMeters()
                    ),
                    new RotationTarget(
                        FlippingUtil.flipFieldRotation(((Waypoint) element).rotationTarget().rotation()),
                        ((Waypoint) element).rotationTarget().t_ratio(),
                        ((Waypoint) element).rotationTarget().profiledRotation()
                    )
                ));
            }
        }

        flipped = true;
    }

    public void undoFlip() {
        if (!isValid()) {
            return;
        }
        
        if (!flipped) return;
        flipped = false;
        flip();
        flipped = false;
    }


    public Path copy() {
        List<PathElement> deepCopiedElements = new ArrayList<>(pathElements.size());
        for (PathElement element : pathElements) {
            deepCopiedElements.add(element.copy());
        }
        return new Path(deepCopiedElements, pathConstraints.copy(), defaultGlobalConstraints.copy());
    }
}   
package frc.robot.lib.auto;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.PropertyNamingStrategies;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.auto.Path.PathElement;
import frc.robot.lib.auto.Path.RotationTarget;
import frc.robot.lib.auto.Path.TranslationTarget;
import frc.robot.lib.auto.Path.Waypoint;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class JsonUtils {
    public static final String PROJECT_ROOT = "src/main/deploy/autos";

    private static final ObjectMapper mapper = new ObjectMapper()
        .setPropertyNamingStrategy(PropertyNamingStrategies.SNAKE_CASE)
        .configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);

    @JsonTypeInfo(use = JsonTypeInfo.Id.NAME, property = "type")
    @JsonSubTypes({
        @JsonSubTypes.Type(value = WaypointDTO.class, name = "waypoint"),
        @JsonSubTypes.Type(value = TranslationTargetDTO.class, name = "translation"),
        @JsonSubTypes.Type(value = RotationTargetDTO.class, name = "rotation")
    })
    private static sealed interface PathElementDTO permits WaypointDTO, TranslationTargetDTO, RotationTargetDTO {}

    // Nested DTOs for waypoint components (no type info needed)
    private static final record NestedTranslationTargetDTO(
        double xMeters,
        double yMeters,
        Double intermediateHandoffRadiusMeters
    ) {
        TranslationTarget toTranslationTarget() {
            return new TranslationTarget(
                new Translation2d(xMeters, yMeters),
                Optional.ofNullable(intermediateHandoffRadiusMeters)
            );
        }
    }

    private static final record NestedRotationTargetDTO(
        double rotationRadians,
        Double tRatio,
        Boolean profiledRotation
    ) {
        RotationTarget toRotationTarget() {
            return new RotationTarget(
                Rotation2d.fromRadians(rotationRadians),
                tRatio == null ? 0.5 : tRatio.doubleValue(),
                Boolean.TRUE.equals(profiledRotation)
            );
        }
    }

    private static final record WaypointDTO(
        NestedTranslationTargetDTO translationTarget,
        NestedRotationTargetDTO rotationTarget
    ) implements PathElementDTO {
        Waypoint toWaypoint() {
            return new Waypoint(
                translationTarget.toTranslationTarget(),
                rotationTarget.toRotationTarget()
            );
        }
    } 

    private static final record TranslationTargetDTO(
        double xMeters,
        double yMeters,
        Double intermediateHandoffRadiusMeters
    ) implements PathElementDTO {
        TranslationTarget toTranslationTarget() {
            return new TranslationTarget(
                new Translation2d(xMeters, yMeters),
                Optional.ofNullable(intermediateHandoffRadiusMeters)
            );
        }
    }

    private static final record RotationTargetDTO(
        double rotationRadians,
        Double tRatio,
        Boolean profiledRotation
    ) implements PathElementDTO {
        RotationTarget toRotationTarget() {
            return new RotationTarget(
                Rotation2d.fromRadians(rotationRadians),
                tRatio == null ? 0.5 : tRatio.doubleValue(),
                Boolean.TRUE.equals(profiledRotation)
            );
        }
    }

    private static final record RangedConstraintDTO(
        double value,
        int startOrdinal,
        int endOrdinal
    ) {}

    @JsonIgnoreProperties(ignoreUnknown = true)
    private static final record AutosPathDTO(
        Map<String, List<RangedConstraintDTO>> rangedConstraints,
        List<PathElementDTO> pathElements
    ) {}

    @JsonIgnoreProperties(ignoreUnknown = true)
    private static final record AutosConfigDTO(
        double robotLengthMeters,
        double robotWidthMeters,
        double defaultMaxVelocityMetersPerSec,
        double defaultMaxAccelerationMetersPerSec2,
        double defaultMaxVelocityDegPerSec,
        double defaultMaxAccelerationDegPerSec2,
        double defaultEndTranslationToleranceMeters,
        double defaultEndRotationToleranceDeg,
        double defaultIntermediateHandoffRadiusMeters
    ) {}

    public static <T> T loadFromFile(File file, TypeReference<T> type) {
        try {
            return mapper.readValue(file, type);
        } catch (IOException e) {
            throw new RuntimeException("Failed to load JSON from " + file.getPath(), e);
        }
    }

    public static <T> T loadFromString(String json, TypeReference<T> type) {
        try {
            return mapper.readValue(json, type);
        } catch (IOException e) {
            throw new RuntimeException("Failed to parse JSON string", e);
        }
    }

    public static List<PathElement> loadPathElements(File autosPathFile) {
        AutosPathDTO dto = loadFromFile(autosPathFile, new TypeReference<AutosPathDTO>() {});
        List<PathElement> out = new ArrayList<>();
        for (PathElementDTO e : dto.pathElements()) {
            if (e instanceof WaypointDTO w) out.add(w.toWaypoint());
            else if (e instanceof TranslationTargetDTO t) out.add(t.toTranslationTarget());
            else if (e instanceof RotationTargetDTO r) out.add(r.toRotationTarget());
            else throw new IllegalArgumentException("Unknown PathElementDTO type: " + e.getClass().getName());
        }
        return out;
    }

    public static Path.DefaultGlobalConstraints loadGlobalConstraints(File autosDir) {
        File config = new File(autosDir, "config.json");
        AutosConfigDTO cfg = loadFromFile(config, new TypeReference<AutosConfigDTO>() {});
        return new Path.DefaultGlobalConstraints(
            cfg.defaultMaxVelocityMetersPerSec(),
            cfg.defaultMaxAccelerationMetersPerSec2(),
            cfg.defaultMaxVelocityDegPerSec(),
            cfg.defaultMaxAccelerationDegPerSec2(),
            cfg.defaultEndTranslationToleranceMeters(),
            cfg.defaultEndRotationToleranceDeg(),
            cfg.defaultIntermediateHandoffRadiusMeters()
        );
    }

    public static Path.PathConstraints loadPathConstraints(File autosPathFile) {
        AutosPathDTO dto = loadFromFile(autosPathFile, new TypeReference<AutosPathDTO>() {});
        Path.PathConstraints constraints = new Path.PathConstraints();
        Map<String, List<RangedConstraintDTO>> rc = dto.rangedConstraints();
        if (rc != null) {
            List<RangedConstraintDTO> v;
            v = rc.get("max_velocity_meters_per_sec");
            if (v != null && !v.isEmpty()) {
                ArrayList<Path.RangedConstraint> list = v.stream()
                    .map(rcDto -> new Path.RangedConstraint(rcDto.value(), rcDto.startOrdinal(), rcDto.endOrdinal()))
                    .collect(java.util.stream.Collectors.toCollection(ArrayList::new));
                constraints.setMaxVelocityMetersPerSec(Optional.of(list));
            }
            v = rc.get("max_acceleration_meters_per_sec2");
            if (v != null && !v.isEmpty()) {
                ArrayList<Path.RangedConstraint> list = v.stream()
                    .map(rcDto -> new Path.RangedConstraint(rcDto.value(), rcDto.startOrdinal(), rcDto.endOrdinal()))
                    .collect(java.util.stream.Collectors.toCollection(ArrayList::new));
                constraints.setMaxAccelerationMetersPerSec2(Optional.of(list));
            }
            v = rc.get("max_velocity_deg_per_sec");
            if (v != null && !v.isEmpty()) {
                ArrayList<Path.RangedConstraint> list = v.stream()
                    .map(rcDto -> new Path.RangedConstraint(rcDto.value(), rcDto.startOrdinal(), rcDto.endOrdinal()))
                    .collect(java.util.stream.Collectors.toCollection(ArrayList::new));
                constraints.setMaxVelocityDegPerSec(Optional.of(list));
            }
            v = rc.get("max_acceleration_deg_per_sec2");
            if (v != null && !v.isEmpty()) {
                ArrayList<Path.RangedConstraint> list = v.stream()
                    .map(rcDto -> new Path.RangedConstraint(rcDto.value(), rcDto.startOrdinal(), rcDto.endOrdinal()))
                    .collect(java.util.stream.Collectors.toCollection(ArrayList::new));
                constraints.setMaxAccelerationDegPerSec2(Optional.of(list));
            }
        }
        return constraints;
    }

    public static Path loadPath(File autosDir, String pathFileName) {
        File pathFile = new File(new File(autosDir, "paths"), pathFileName);
        List<PathElement> elements = loadPathElements(pathFile);
        Path.PathConstraints constraints = loadPathConstraints(pathFile);
        Path.DefaultGlobalConstraints globals = loadGlobalConstraints(autosDir);
        return new Path(elements, constraints, globals);
    }

    public static Path loadPath(String pathFileName) {
        return loadPath(new File(PROJECT_ROOT), pathFileName);
    }
}

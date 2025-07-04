package frc.robot.lib.auto;

import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.auto.Path.PathElement;
import frc.robot.lib.auto.Path.RotationTarget;
import frc.robot.lib.auto.Path.TranslationTarget;
import frc.robot.lib.auto.Path.Waypoint;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Optional;

public class JsonUtils {
    private static final ObjectMapper mapper = new ObjectMapper();

    @JsonTypeInfo(use = JsonTypeInfo.Id.NAME, property = "type")
    @JsonSubTypes({
        @JsonSubTypes.Type(value = WaypointDTO.class, name = "waypoint"),
        @JsonSubTypes.Type(value = TranslationElementDTO.class, name = "translation"),
        @JsonSubTypes.Type(value = RotationElementDTO.class, name = "rotation")
    })
    private static sealed interface PathElementDTO permits WaypointDTO, TranslationElementDTO, RotationElementDTO {}

    private static final record WaypointDTO(
        double xMeters,
        double yMeters, 
        Double finalVelocityMetersPerSec, 
        Double maxVelocityMetersPerSec, 
        Double maxAccelerationMetersSPerSec2,
        Double intermediateHandoffRadiusMeters,

        double rotationDegrees,
        Double maxVelocityDegPerSec,
        Double maxAccelerationDegSPerSec2
    ) implements PathElementDTO {
        Waypoint toWaypoint() {
            return new Waypoint(
                new TranslationTarget(
                    new Translation2d(xMeters, yMeters), 
                    Optional.ofNullable(finalVelocityMetersPerSec), 
                    Optional.ofNullable(maxVelocityMetersPerSec), 
                    Optional.ofNullable(maxAccelerationMetersSPerSec2), 
                    Optional.ofNullable(intermediateHandoffRadiusMeters)
                ),
                new RotationTarget(
                    Rotation2d.fromDegrees(rotationDegrees), 
                    new Translation2d(xMeters, yMeters), 
                    Optional.ofNullable(maxVelocityDegPerSec == null ? null : Double.valueOf(Math.toRadians(maxVelocityDegPerSec))),
                    Optional.ofNullable(maxAccelerationDegSPerSec2 == null ? null : Double.valueOf(Math.toRadians(maxAccelerationDegSPerSec2)))
                )
            );
        }
    } 

    private static final record TranslationElementDTO(
        double xMeters,
        double yMeters, 
        Double finalVelocityMetersPerSec, 
        Double maxVelocityMetersPerSec, 
        Double maxAccelerationMetersSPerSec2,
        Double intermediateHandoffRadiusMeters
    ) implements PathElementDTO {
        TranslationTarget toTranslationTarget() {
            return new TranslationTarget(
                new Translation2d(xMeters, yMeters),
                Optional.ofNullable(finalVelocityMetersPerSec),
                Optional.ofNullable(maxVelocityMetersPerSec),
                Optional.ofNullable(maxAccelerationMetersSPerSec2),
                Optional.ofNullable(intermediateHandoffRadiusMeters)
            );
        }
    }

    private static final record RotationElementDTO(
        double rotationDegrees,
        double xMeters,
        double yMeters,
        Double maxVelocityDegPerSec,
        Double maxAccelerationDegSPerSec2
    ) implements PathElementDTO {
        RotationTarget toRotationTarget() {
            return new RotationTarget(
                Rotation2d.fromDegrees(rotationDegrees), 
                new Translation2d(xMeters, yMeters), 
                Optional.ofNullable(maxVelocityDegPerSec == null ? null : Double.valueOf(Math.toRadians(maxVelocityDegPerSec))),
                Optional.ofNullable(maxAccelerationDegSPerSec2 == null ? null : Double.valueOf(Math.toRadians(maxAccelerationDegSPerSec2)))
            );
        }
    }

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

    public static List<PathElement> loadPathElements(File file) {
        return loadFromFile(file, new TypeReference<List<PathElementDTO>>() {}).stream().map(
            dto -> {
                if (dto instanceof WaypointDTO w) return (PathElement) w.toWaypoint();
                if (dto instanceof TranslationElementDTO t) return (PathElement) t.toTranslationTarget();
                if (dto instanceof RotationElementDTO r) return (PathElement) r.toRotationTarget();
                throw new IllegalArgumentException("Unknown PathElementDTO type: " + dto.getClass().getName());
            }
        ).toList();
    }
}

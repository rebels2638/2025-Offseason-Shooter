package frc.robot.lib.auto;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.auto.FollowPath.Waypoint;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Optional;

public class JsonUtils {
    private static final ObjectMapper mapper = new ObjectMapper();
    
    private static final record WaypointDTO(double x, double y, Double rotation, Double velocity) {
        Waypoint toWaypoint() {
            return new Waypoint(
                new Translation2d(x, y), 
                Optional.ofNullable(rotation).map(Rotation2d::fromDegrees),
                Optional.ofNullable(velocity)
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

    public static List<Waypoint> loadWaypoints(File file) {
        return loadFromFile(file, new TypeReference<List<WaypointDTO>>() {}).stream().map(WaypointDTO::toWaypoint).toList();
    }
}

package frc.robot.constants.vision;

import java.io.File;
import java.io.IOException;
import java.util.logging.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout;
  private static final Logger logger = Logger.getLogger(VisionConstants.class.getName());

  static {
    try {
      File layoutFile = new File(Filesystem.getDeployDirectory(), "custom_apriltag_field_layout.json");
      aprilTagLayout = new AprilTagFieldLayout(layoutFile.getAbsolutePath());
      logger.info("Loaded custom AprilTag field layout from deploy directory");
    } catch (IOException e) {
      // Fallback to default field layout if custom field fails to load
      aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
      logger.severe("AprilTagFieldLayout failed to load: " + e.getMessage());
      e.printStackTrace();
    }
  }

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 =
        new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
    public static Transform3d robotToCamera1 =
        new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations. less than 1 means more stable than full 3D solve (more weight)
  public static double linearStdDevMegatag2Factor = Double.POSITIVE_INFINITY;; // More stable than full 3D solve but we reject for now
  public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

  // Rotation rate rejection thresholds
  public static double maxRotationRateMegatag1DegreesPerSecond = 180.0; // Reject MegaTag1 observations if rotating faster than this
  public static double maxRotationRateMegatag2DegreesPerSecond = 120.0; // Reject MegaTag2 observations if rotating faster than this
}
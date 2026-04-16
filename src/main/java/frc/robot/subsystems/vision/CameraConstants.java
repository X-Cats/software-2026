package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.List;

/**
 * Constants and helpers for camera configuration (FOV, std-dev coefficients, and robot camera
 * list).
 */
public class CameraConstants {
  /** Approximate LED blink period in seconds (utility for UI/debug). */
  public static final double BLINK_TIME = 0.067;

  /** Limelight 2+ nominal parameters. */
  public static class Limelight2PlusConstants {
    public static final double HORIZONTAL_FOV = Units.degreesToRadians(62.5);
    public static final double VERTICAL_FOV = Units.degreesToRadians(48.9);
    public static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    public static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    public static final double COMPLEMENTARY_FILTER_SIGMA = 0.5;
  }

  /** Limelight 3 nominal parameters. */
  public static class Limelight3Constants {
    public static final double HORIZONTAL_FOV = Units.degreesToRadians(62.5);
    public static final double VERTICAL_FOV = Units.degreesToRadians(48.9);
    public static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.05;
    public static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.05;
  }

  /** Limelight 3G nominal parameters. */
  public static class Limelight3GConstants {
    public static final double HORIZONTAL_FOV = Units.degreesToRadians(82.0);
    public static final double VERTICAL_FOV = Units.degreesToRadians(46.2);
    public static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.05;
    public static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.00015;
  }

  /** Limelight 4 nominal parameters. */
  public static class Limelight4Constants {
    public static final double HORIZONTAL_FOV = Units.degreesToRadians(82.0);
    public static final double VERTICAL_FOV = Units.degreesToRadians(46.2);
    public static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.05;
    public static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.00015;
  }

  /** Factory for robot cameras. Edit transforms and duties to match your robot configuration. */
  public static class RobotCameras {
    /** Left-side Limelight configuration including physical transform and duties. */
    public static final Camera LEFT =
        new Camera(
            new CameraIOLimelight("left", CameraType.LIMELIGHT_4),
            Limelight4Constants.HORIZONTAL_FOV,
            Limelight4Constants.VERTICAL_FOV,
            Limelight4Constants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight4Constants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault()
                .getTable("limelight-left")
                .getDoubleArrayTopic("robot_orientation_set")
                .publish(),
            List.of(CameraDuty.FIELD_LOCALIZATION),
            new Transform3d(
                -0.33, // forward + / backward -
                -0.089, // side right + / left -
                0.419, // up + / down -
                new Rotation3d(
                    Units.degreesToRadians(3.45),
                    Units.degreesToRadians(23.98),
                    Units.degreesToRadians(170.0))));

    /** Right-side Limelight configuration including physical transform and duties. */
    public static final Camera RIGHT =
        new Camera(
            new CameraIOLimelight("right", CameraType.LIMELIGHT_4),
            Limelight4Constants.HORIZONTAL_FOV,
            Limelight4Constants.VERTICAL_FOV,
            Limelight4Constants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight4Constants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault()
                .getTable("limelight-right")
                .getDoubleArrayTopic("robot_orientation_set")
                .publish(),
            List.of(CameraDuty.FIELD_LOCALIZATION),
            new Transform3d(
                -0.33, // forward + / backward -
                0.089, // side right + / left -
                0.419, // up + / down -
                new Rotation3d(
                    Units.degreesToRadians(-4.97),
                    Units.degreesToRadians(24.77),
                    Units.degreesToRadians(190.0))));

    /** Both comp bot cameras used for field localization. */
    public static final Camera[] CAMERAS = {LEFT, RIGHT};
  }

  /** Placeholder for replay/sim configurations if needed. */
  public static class ReplayCameras {}
}

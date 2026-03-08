package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class LauncherConstants {
  public static Transform3d robotToLauncher =
      new Transform3d(-0.205, 0.0, 0.474191, new Rotation3d(0.0, 0.0, 0.0));

  private LauncherConstants() {}
}

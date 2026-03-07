package frc.robot.subsystems.HoodKicker;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;

public class LaunchCalculator {

  private Supplier<Pose2d> poseSupplier;

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d driveAngle,
      double driveVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight,
      boolean passing) {}

  private LaunchingParameters latestParameters = null;

  public LaunchingParameters getParameters() {
    boolean passing =
            org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil.applyX(
                    poseSupplier.get().getX())
                    > org.littletonrobotics.frc2026.FieldConstants.LinesVertical.hubCenter;
    if (latestParameters != null) {
      return latestParameters;
    }

  }

}


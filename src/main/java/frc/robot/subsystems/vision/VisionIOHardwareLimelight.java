package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Localization;
import frc.robot.util.limelight.LimelightHelpers;
import java.util.concurrent.atomic.AtomicReference;

public class VisionIOHardwareLimelight implements VisionIO {
  NetworkTable table =
      NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightTableName);
  NetworkTable tableB =
      NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightBTableName);

  Localization localization;
  AtomicReference<VisionIOInputs> latestInputs = new AtomicReference<>(new VisionIOInputs());

  public VisionIOHardwareLimelight(Localization localization) {
    this.localization = localization;
    setLLSettings();
  }

  private void setLLSettings() {
    double[] camerapose = {
      0.0,
      0.0,
      VisionConstants.kCameraHeightOffGroundMeters,
      VisionConstants.kCameraRollDegrees,
      VisionConstants.kCameraHeightOffGroundMeters,
      0.0
    };
    tableB.getEntry("camerapose_robotspace_set").setDoubleArray(camerapose);

    double[] cameraBpose = {
      0.0,
      0.0,
      VisionConstants.kCameraBHeightOffGroundMeters,
      VisionConstants.kCameraBRollDegrees,
      VisionConstants.kCameraBHeightOffGroundMeters,
      0.0
    };
    tableB.getEntry("camerapose_robotspace_set").setDoubleArray(camerapose);

    var gyroAngle = localization.getLatestFieldToRobot().getValue().getRotation();
    var gyroAngularVelocity =
        Units.radiansToDegrees(
            localization.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond);
    LimelightHelpers.SetRobotOrientation(
        VisionConstants.kLimelightBTableName,
        gyroAngle.getDegrees(),
        gyroAngularVelocity,
        0,
        0,
        0,
        0);
  }

  @Override
  public void readInputs(VisionIOInputs inputs) {
    inputs.camera1SeesTarget = table.getEntry("tv").getDouble(0) == 1.0;
    inputs.camera2SeesTarget = tableB.getEntry("tv").getDouble(0) == 1.0;
    if (inputs.camera1SeesTarget) {
      var megatag =
          LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.kLimelightTableName);
      var megatag2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kLimelightTableName);
      inputs.camera1MegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
      inputs.camera1MegatagCount = megatag.tagCount;
      inputs.camera1Megatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
      inputs.camera1FiducialObservations = FiducialObservation.fromLimelight(megatag.rawFiducials);
    }
    if (inputs.camera2SeesTarget) {
      var megatag =
          LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.kLimelightBTableName);
      var megatag2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
              VisionConstants.kLimelightBTableName);
      inputs.camera2MegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
      inputs.camera2MegatagCount = megatag.tagCount;
      inputs.camera2Megatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
      inputs.camera2FiducialObservations = FiducialObservation.fromLimelight(megatag.rawFiducials);
    }
    latestInputs.set(inputs);
    setLLSettings();
  }

  @Override
  public void pollNetworkTables() {
    VisionIOInputs inputs = new VisionIOInputs();

    // See if we see the target
    inputs.camera1SeesTarget = table.getEntry("tv").getDouble(0) == 1.0;
    inputs.camera2SeesTarget = tableB.getEntry("tv").getDouble(0) == 1.0;
    if (inputs.camera1SeesTarget) {
      var megatag =
          LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.kLimelightTableName);
      var megatag2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kLimelightTableName);
      inputs.camera1MegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
      inputs.camera1MegatagCount = megatag.tagCount;
      inputs.camera1Megatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
      inputs.camera1FiducialObservations = FiducialObservation.fromLimelight(megatag.rawFiducials);
    }
    if (inputs.camera2SeesTarget) {
      var megatag =
          LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.kLimelightBTableName);
      var megatag2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
              VisionConstants.kLimelightBTableName);
      inputs.camera2MegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
      inputs.camera2MegatagCount = megatag.tagCount;
      inputs.camera2Megatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
      inputs.camera2FiducialObservations = FiducialObservation.fromLimelight(megatag.rawFiducials);
    }
    latestInputs.set(inputs);
    setLLSettings();
  }
}

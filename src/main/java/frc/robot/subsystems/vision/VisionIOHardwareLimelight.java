package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Localization;
import frc.robot.util.limelight.LimelightHelpers;

import java.util.concurrent.atomic.AtomicReference;

public class VisionIOHardwareLimelight implements VisionIO{
    NetworkTable table = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightTableName);
    NetworkTable tableB = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightBTableName);

    Localization localization;
    AtomicReference<VisionIOInputs> latestInputs = new AtomicReference<>(new VisionIOInputs());

    public VisionIOHardwareLimelight(Localization localization) {
        this.localization = localization;
        setLLSettings();
    }

    private void setLLSettings() {
        double[] camerapose = { 0.0, 0.0, VisionConstants.kCameraHeightOffGroundMeters, VisionConstants.kCameraRollDegrees,
                VisionConstants.kCameraHeightOffGroundMeters, 0.0 };
        tableB.getEntry("camerapose_robotspace_set").setDoubleArray(camerapose);

        double[] cameraBpose = { 0.0, 0.0, VisionConstants.kCameraBHeightOffGroundMeters, VisionConstants.kCameraBRollDegrees,
                VisionConstants.kCameraBHeightOffGroundMeters, 0.0 };
        tableB.getEntry("camerapose_robotspace_set").setDoubleArray(camerapose);

        var gyroAngle = localization.getLatestFieldToRobot().getValue().getRotation();
        var gyroAngularVelocity = units
                .radiansToDegrees(localization.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond);
        LimelightHelpers.SetRobotOrientation(VisionConstants.kLimelightBTableName, gyroAngle.getDegrees(),
                gyroAngularVelocity, 0, 0, 0, 0);
    }
}

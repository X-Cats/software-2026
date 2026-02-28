package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Localization;
import frc.robot.util.RobotTime;
import jdk.jshell.execution.Util;
import org.littletonrobotics.junction.Logger;
import java.util.

public class VisionSubsystem extends SubsystemBase {
    private final VisionIO io;

    private final Localization localization;

    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    private static class PinholeObservation {
        public Translation2d cameraToTag;
        public Pose3d tagPose;
    }

    private double lastProcessedCamera1Timestamp = 0.0;
    private double lastProcessedCamera2Timestamp = 0.0;

    public VisionSubsystem(VisionIO io, Localization localization) {
        this.io = io;
        this.localization = localization;
    }

    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds();
        // Read inputs from IO
        io.readInputs(inputs);
        Logger.processInputs("Vision", inputs);

        //Updates localization
        if (inputs.camera1SeesTarget) {
            updateVision(inputs.camera1SeesTarget, inputs.camera1FiducialObservations,
                    inputs.camera1MegatagPoseEstimate, inputs.camera1Megatag2PoseEstimate, true);
        } else if (inputs.camera2SeesTarget) {
            updateVision(inputs.camera2SeesTarget, inputs.camera2FiducialObservations,
                    inputs.camera2MegatagPoseEstimate, inputs.camera2Megatag2PoseEstimate, false);
        }

        Logger.recordOutput("Vision/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
    }

    private void updateVision(boolean cameraSeesTarget, FiducialObservation[] cameraFiducialObservations,
                              MegatagPoseEstimate cameraMegatagPoseEstimate, MegatagPoseEstimate cameraMegatag2PoseEstimate,
                              boolean isCamera1) {
        if (cameraMegatagPoseEstimate != null) {

            String logPreface = "Vision/" + (isCamera1 ? "Camera1/" : "Camera2/");
            var updateTimestamp = cameraMegatagPoseEstimate.timestampSeconds;
            boolean alreadyProcessedTimestamp = (isCamera1 ? lastProcessedCamera1Timestamp
                    : lastProcessedCamera2Timestamp) == updateTimestamp;
            if (!alreadyProcessedTimestamp && cameraSeesTarget) {
                Optional<VisionFieldPoseEstimate> poseEstimate = Optional.empty();
            }
        }
    }
}

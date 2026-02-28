package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  class VisionIOInputs {
    // TODO finalize actual limelight camera names
    public boolean camera1SeesTarget;

    public boolean camera2SeesTarget;

    public FiducialObservation[] camera1FiducialObservations;

    public FiducialObservation[] camera2FiducialObservations;

    public MegatagPoseEstimate camera1MegatagPoseEstimate;

    public int camera1MegatagCount;

    public MegatagPoseEstimate camera2MegatagPoseEstimate;

    public int camera2MegatagCount;

    public MegatagPoseEstimate camera1Megatag2PoseEstimate;

    public MegatagPoseEstimate camera2Megatag2PoseEstimate;
  }

  void readInputs(VisionIOInputs inputs);

  void pollNetworkTables();
}

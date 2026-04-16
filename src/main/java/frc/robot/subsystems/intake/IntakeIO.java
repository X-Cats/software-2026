package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double rollerAppliedVolts = 0.0;
    public double rollerVelocity = 0.0;
    public double rollerTorqueCurrentAmps = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;
    public double deployIn = 0.0;
    public double deployOut = 0.0;

    public double deployAppliedVolts = 0.0;
    public double deployVelocity = 0.0;
    public double deployPosition = 0.0;
    public double deployTorqueCurrentAmps = 0.0;
    public double deploySupplyCurrentAmps = 0.0;
    public double deploySetpoint = 0.0;
    public double deployAtSetpoint = 0;
  }

  public static class IntakeIOTunables {
    public double deploymentKV,
        deploymentKS,
        deploymentKA,
        deploymentKP,
        deploymentKI,
        deploymentKD,
        deploymentMMCruiseVelocity,
        deploymentMMAcceleration,
        deploymentMMJerk = 0.0;
  }

  public static class IntakeIOTunables {
    public double deploymentKV,
        deploymentKS,
        deploymentKA,
        deploymentKP,
        deploymentKI,
        deploymentKD,
        deploymentMMCruiseVelocity,
        deploymentMMAcceleration,
        deploymentMMJerk = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void applyTunables(IntakeIOTunables tunables) {}

  /** sets the deployment motor's current */
  public default void setDeployMotorTorque(double amps) {}

  public default void setDeployMotorPosition(double ticks) {}
  /**
   * sets the roller motor's voltage
   *
   * @param volts voltage value from -12 to 12
   */
  public default void setRollerMotorVoltage(double volts) {}

  /**
   * Sets the roller motor's torque
   *
   * <p>TODO: What is the torque unit?
   */
  public default void setRollerMotorTorque(double torque) {}

  public default void setRollerMotorSpeed(double velocityRpm) {}

  public default void zeroDeploy() {}
}

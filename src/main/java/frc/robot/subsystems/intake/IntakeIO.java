package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double rollerAppliedVolts = 0.0;
    public double rollerVelocity = 0.0;
    public double rollerTorqueCurrentAmps = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;

    public double deployAppliedVolts = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  /**
   * sets the deployment motor's current
   *
   */
  public default void setDeployMotorTorque(double amps) {}
  /**
   * sets the roller motor's voltage
   *
   * @param volts voltage value from -12 to 12
   */
  public default void setRollerMotorVoltage(double volts) {}

  /**
   * Sets the roller motor's torque
   *
   * TODO: What is the torque unit?
   */
  public default void setRollerMotorTorque(double torque) {}

}

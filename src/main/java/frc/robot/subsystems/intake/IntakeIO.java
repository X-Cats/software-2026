package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double rollerAppliedVolts = 0.0;

    public double deployAppliedVolts = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  /**
   * sets the deployment motor's voltage
   *
   * @param volts voltage value from -12 to 12
   */
  public default void setDeploymentMotorVoltage(double volts) {}
  /**
   * sets the roller motor's voltage
   *
   * @param volts voltage value from -12 to 12
   */
  public default void setRollerMotorVoltage(double volts) {}
}

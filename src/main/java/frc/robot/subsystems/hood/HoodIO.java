package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public double hoodAppliedVolts = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  /**
   * setting the hood motor voltage
   *
   * @param volts voltage to set from -12 to 12
   */
  public default void setHoodMotorVoltage(double volts) {}
}

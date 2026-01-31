package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public double hopperAppliedVolts = 0.0;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void setHopperMotorVoltage(double volts) {}
}

package frc.robot.subsystems.HoodKicker;

import org.littletonrobotics.junction.AutoLog;

public interface HoodKickerIO {
  @AutoLog
  public static class HoodIOInputs {
    public double hoodAppliedVolts = 0.0;
    public double hoodVelocity = 0.0;
    public double hoodPosition = 0.0;
    public double hoodTorqueCurrent = 0.0;
    public double hoodSupplyCurrent = 0.0;
    public double hoodForwardLimit = 0.0;
    public double hoodReverseLimit = 0.0;

    public double kickerAppliedVolts = 0.0;
    public double kickerVelocity = 0.0;
    public double kickerTorqueCurrent = 0.0;
    public double kickerSupplyCurrent = 0.0;
  }

  @AutoLog
  public static class HoodIOOutputs {
    public boolean hasBeenZeroed = false;
    public double positionRotations = 0.0;
    public double velocityRadPerSecond = 0.0;
    public double kP = 0.0;
    public double kD = 0.0;
    public double kS = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void applyOutputs(HoodIOOutputs outputs) {}

  public default void applyTunables(HoodIOOutputs outputs) {}

  /**
   * setting the hood motor voltage
   *
   * @param volts voltage to set from -12 to 12
   */
  public default void setHoodMotorVoltage(double volts) {}

  public default void setHoodPosition(double ticks) {}

  /**
   * setting the kicker motor voltage
   *
   * @param volts voltage to set from
   */
  public default void setKickerMotorVoltage(double volts) {}

  public default void zero() {}
}

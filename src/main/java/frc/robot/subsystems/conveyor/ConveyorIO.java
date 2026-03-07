package frc.robot.subsystems.conveyor;

import org.littletonrobotics.junction.AutoLog;

public interface ConveyorIO {
  @AutoLog
  public static class ConveyorIOInputs {
    public double conveyorAppliedVolts = 0.0;
    public double conveyorSupplyCurrent = 0.0;
    public double conveyorStatorCurrent = 0.0;
  }

  public default void updateInputs(ConveyorIOInputs inputs) {}

  public default void applyOutputs() {}

  /**
   * setting the hopper motor voltage
   *
   * @param amps amperage to set
   */
  public default void setConveyorVoltage(double amps) {}
}

package frc.robot.subsystems.conveyor;

import org.littletonrobotics.junction.AutoLog;

public interface ConveyorIO {
  @AutoLog
  public static class ConveyorIOInputs {
    public double conveyorAppliedVolts = 0.0;
  }

  public default void updateInputs(ConveyorIOInputs inputs) {}

  /**
   * setting the hopper motor voltage
   *
   * @param amps amperage to set
   */
  public default void setConveyorTorque(double amps) {}
}

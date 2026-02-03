package frc.robot.subsystems.conveyor;

import org.littletonrobotics.junction.AutoLog;

public interface ConveyorIO {
  @AutoLog
  public static class ConveyorIOInputs {
    public double conveyorAppliedVolts = 0.0;
  }

  public default void updateInputs(ConveyorIOInputs inputs) {}

  public default void setConveyorMotorVoltage(double volts) {}
}

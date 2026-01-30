package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double rollerAppliedVolts = 0.0;

    public double deployAppliedVolts = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setDeploymentMotorVoltage(double volts) {}

  public default void setRollerMotorVoltage(double volts) {}
  
  public void close();
}

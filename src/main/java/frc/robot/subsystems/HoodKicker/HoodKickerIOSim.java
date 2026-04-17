package frc.robot.subsystems.HoodKicker;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodKickerIOSim implements HoodKickerIO {
  private DCMotorSim hoodSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44Foc(1), 0.004, HoodKickerConstants.HOOD_MOTOR_REDUCTION),
          DCMotor.getKrakenX44Foc(1));
  private DCMotorSim kickerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44Foc(1), 0.004, HoodKickerConstants.KICKER_MOTOR_REDUCTION),
          DCMotor.getKrakenX44Foc(1));

  private double hoodAppliedVolts = 0.0;
  private double kickerAppliedVolts = 0.0;
  private double hoodPosition = 0.0;

  public void updateInputs(HoodIOInputs inputs) {
    hoodSim.setInputVoltage(hoodAppliedVolts);
    hoodSim.update(0.02);

    inputs.hoodAppliedVolts = hoodAppliedVolts;
    inputs.hoodPosition = hoodPosition;

    kickerSim.setInputVoltage(kickerAppliedVolts);
    kickerSim.update(0.02);

    inputs.kickerAppliedVolts = kickerAppliedVolts;
  }

  public void applyOutputs(HoodIOOutputs outputs) {
    hoodPosition = outputs.positionRotations;
  }

  @Override
  public void setHoodMotorVoltage(double volts) {
    hoodAppliedVolts = MathUtil.clamp(volts, -12.0, 12);
  }

  @Override
  public void setKickerMotorVoltage(double volts) {
    kickerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}

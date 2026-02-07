package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIOSim implements HoodIO {
  private DCMotorSim hoodSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44Foc(1), 0.004, HoodConstants.HOOD_MOTOR_REDUCTION),
          DCMotor.getKrakenX44Foc(1));

  private double hoodAppliedVolts = 0.0;

  public void updateInputs(HoodIOInputs inputs) {
    hoodSim.setInputVoltage(hoodAppliedVolts);
    hoodSim.update(0.02);

    inputs.hoodAppliedVolts = hoodAppliedVolts;
  }

  @Override
  public void setHoodMotorVoltage(double volts) {
    hoodAppliedVolts = MathUtil.clamp(volts, -12.0, 12);
  }
}

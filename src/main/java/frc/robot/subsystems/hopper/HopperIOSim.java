package frc.robot.subsystems.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HopperIOSim implements HopperIO {
  private DCMotorSim hopperSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44Foc(1), 0.004, HopperConstants.HOPPER_MOTOR_REDUCTION),
          DCMotor.getKrakenX44Foc(1));

  private double hopperAppliedVolts = 0.0;

  public void updateInputs(HopperIOInputs inputs) {
    hopperSim.setInputVoltage(hopperAppliedVolts);
    hopperSim.update(0.02);

    inputs.hopperAppliedVolts = hopperAppliedVolts;
  }

  @Override
  public void setHopperMotorVoltage(double volts) {
    hopperAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}

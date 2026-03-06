package frc.robot.subsystems.conveyor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ConveyorIOSim implements ConveyorIO {
  private DCMotorSim hopperSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44Foc(1), 0.004, ConveyorConstants.CONVEYOR_MOTOR_REDUCTION),
          DCMotor.getKrakenX44Foc(1));

  private double conveyorAppliedVolts = 0.0;

  public void updateInputs(ConveyorIOInputs inputs) {
    hopperSim.setInputVoltage(conveyorAppliedVolts);
    hopperSim.update(0.02);

    inputs.conveyorAppliedVolts = conveyorAppliedVolts;
  }

  @Override
  public void setConveyorVoltage(double volts) {
    conveyorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}

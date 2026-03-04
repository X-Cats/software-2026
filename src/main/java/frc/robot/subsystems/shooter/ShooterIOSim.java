package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {
  private PIDController controller = new PIDController(0.6, 0, 0, Constants.loopPeriodSecs);
  private DCMotorSim shooterSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44Foc(2), 0.004, ShooterConstants.SHOOTER_MOTOR_REDUCTION),
          DCMotor.getKrakenX44Foc(2));

  private double shooterAppliedVelocity = 0.0;
  private double shooterCurrentOutput = 0.0;
  private double shooterLastVelocity = 0.0;

  public void updateInputs(ShooterIOInputs inputs) {
    shooterSim.setInputVoltage(shooterAppliedVelocity);
    shooterSim.update(0.02);

    shooterLastVelocity = inputs.radsPerSecond;
    inputs.shooterAppliedVolts = shooterSim.getAngularVelocityRadPerSec();
  }

  public void setShooterMotorVoltage(double volts) {
    shooterAppliedVelocity = MathUtil.clamp(volts, -12.0, 12.0);
  }

  // TODO: Implement applyOutputs
  @Override
  public void applyOutputs(ShooterIOOutputs outputs) {
    if (outputs.mode == ShooterIOOutputMode.COAST) {
      shooterCurrentOutput = 0.0;
    } else {
      shooterCurrentOutput = controller.calculate(shooterLastVelocity);
      controller.setSetpoint(outputs.velocityRadsPerSec);
    }
  }
}

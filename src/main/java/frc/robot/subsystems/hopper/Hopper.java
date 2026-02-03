package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  }

  public Command runHopperMotor() {
    return runEnd(
        () -> {
          io.setHopperMotorVoltage(HopperConstants.HOPPER_MOTOR_VOLTAGE);
        },
        () -> {
          io.setHopperMotorVoltage(0.0);
        });
  }
}

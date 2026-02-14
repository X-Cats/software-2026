package frc.robot.subsystems.HoodKicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class HoodKicker extends SubsystemBase {
  private final HoodKickerIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public HoodKicker(HoodKickerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public Command runHoodMotor() {
    return runEnd(
        () -> {
          io.setHoodMotorVoltage(HoodKickerConstants.HOOD_MOTOR_VOLTAGE);
        },
        () -> {
          io.setHoodMotorVoltage(0.0);
        });
  }

  public Command runKickerMotor() {
    return runEnd(
            () -> {
              io.setKickerMotorVoltage(HoodKickerConstants.KICKER_MOTOR_VOLTAGE);
            },
            () -> {
              io.setKickerMotorVoltage(0.0);
            });
  }
}

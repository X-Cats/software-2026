package frc.robot.subsystems.HoodKicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class HoodKicker extends SubsystemBase {
  private final HoodKickerIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final RobotState robotState;

  public HoodKicker(HoodKickerIO io, RobotState rs) {
    this.io = io;
    robotState = rs;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public Command runStateful() {
    return new RunCommand(
        () -> {
          switch (robotState.getDesiredHoodState().getHoodState()) {
            case AIMING -> io.setHoodMotorVoltage(HoodKickerConstants.HOOD_MOTOR_VOLTAGE);
            case STOWED -> io.setHoodMotorVoltage(0);
            default -> {
              System.out.println(
                  "Illegal Hood State : " + robotState.getDesiredHoodState().getHoodState());
              io.setHoodMotorVoltage(0);
            }
          }
        },
        this);
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

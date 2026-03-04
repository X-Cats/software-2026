package frc.robot.subsystems.HoodKicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateMachine;
import org.littletonrobotics.junction.Logger;

public class HoodKicker extends SubsystemBase {
  private final HoodKickerIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final RobotStateMachine robotState;

  private boolean hasBeenZeroed = false;

  public HoodKicker(HoodKickerIO io, RobotStateMachine rs) {
    this.io = io;
    robotState = rs;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    hasBeenZeroed = hasBeenZeroed || -0.5 < inputs.hoodPosition && inputs.hoodPosition < 0.5;
    Logger.processInputs("Hood", inputs);

    if (this.hasBeenZeroed) {
      switch (robotState.getDesiredHoodState().getHoodState()) {
        case AIMING -> io.setHoodPosition(700);
        case STOWED -> io.setHoodPosition(0);
        default -> {
          System.out.println(
              "Illegal Hood State : " + robotState.getDesiredHoodState().getHoodState());
          io.setHoodMotorVoltage(0);
        }
      }
    } else {
      io.zero();
    }

    switch (robotState.getDesiredHoodState().getKickerState()) {
      case FEEDING -> io.setKickerMotorVoltage(HoodKickerConstants.KICKER_MOTOR_VOLTAGE);
      case OFF -> io.setKickerMotorVoltage(0);
    }
  }

  public Command runStateful() {
    return Commands.none();
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

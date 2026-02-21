package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
  private final RobotState robotState;

  public Hopper(HopperIO io, RobotState rs) {
    this.io = io;
    robotState = rs;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  }

  public Command runStateful() {
    return new RunCommand(
        () -> {
          switch (robotState.getDesiredHopperState().getHopperState()) {
            case FEEDING -> io.setHopperMotorVoltage(HopperConstants.HOPPER_MOTOR_VOLTAGE);
              //        case SHUFFLING -> io.
            case EJECTING -> io.setHopperMotorVoltage(-HopperConstants.HOPPER_MOTOR_VOLTAGE);
            case OFF -> io.setHopperMotorVoltage(0);
            default -> {
              System.out.println(
                  "Illegal hopper state : " + robotState.getDesiredHopperState().getHopperState());
              io.setHopperMotorVoltage(0);
            }
          }
        },
        this);
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

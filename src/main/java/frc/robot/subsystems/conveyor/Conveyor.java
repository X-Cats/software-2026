package frc.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateMachine;
import org.littletonrobotics.junction.Logger;

public class Conveyor extends SubsystemBase {
  private final ConveyorIO io;
  private final ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();
  private final RobotStateMachine robotState;

  public Conveyor(ConveyorIO io, RobotStateMachine rs) {
    this.io = io;
    robotState = rs;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Conveyor", inputs);

    switch (robotState.getDesiredConveyorState().getConveyorState()) {
      case CONVEYING -> io.setConveyorMotorVoltage(ConveyorConstants.CONVEYOR_MOTOR_VOLTAGE);
        //        case SHUFFLING -> io.
      case EJECTING -> io.setConveyorMotorVoltage(-ConveyorConstants.CONVEYOR_MOTOR_VOLTAGE);
      case OFF -> io.setConveyorMotorVoltage(0);
      default -> {
        System.out.println(
            "Illegal conveyor state : " + robotState.getDesiredConveyorState().getConveyorState());
        io.setConveyorMotorVoltage(0);
      }
    }
  }

  public Command runStateful() {
    return Commands.none();
  }

  public Command runConveyorMotor() {
    return runEnd(
        () -> {
          io.setConveyorMotorVoltage(ConveyorConstants.CONVEYOR_MOTOR_VOLTAGE);
        },
        () -> {
          io.setConveyorMotorVoltage(0.0);
        });
  }
}

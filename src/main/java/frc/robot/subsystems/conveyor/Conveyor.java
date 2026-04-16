package frc.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateMachine;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Conveyor extends SubsystemBase {
  private final ConveyorIO io;
  private final ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();
  private final RobotStateMachine robotState;

  private LoggedTunableNumber conveyorVoltage =
      new LoggedTunableNumber("Conveyor/Supply Voltage", ConveyorConstants.CONVEYOR_MOTOR_VOLTAGE);

  private LoggedTunableNumber conveyorAgitatingVoltage =
      new LoggedTunableNumber(
          "Conveyor/Agitating Voltage", ConveyorConstants.CONVEYOR_AGITATING_MOTOR_VOLTAGE);

  public Conveyor(ConveyorIO io, RobotStateMachine rs) {
    this.io = io;
    robotState = rs;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Conveyor", inputs);
    io.applyOutputs();

    switch (robotState.getDesiredConveyorState().getConveyorState()) {
      case CONVEYING -> {
        io.setConveyorVoltage(conveyorVoltage.getAsDouble());
      }
      case EJECTING -> io.setConveyorVoltage(-conveyorVoltage.getAsDouble());
      case AGITATING -> {
        if (((int) (Timer.getFPGATimestamp() * 10.0)) % 4
            == 0) { // Every 1/4 of the time we agitate
          io.setConveyorVoltage(conveyorAgitatingVoltage.getAsDouble());
        } else {
          io.setConveyorVoltage(conveyorVoltage.getAsDouble());
        }
      }
      case OFF -> io.setConveyorVoltage(0);
      default -> {
        System.out.println(
            "Illegal conveyor state : " + robotState.getDesiredConveyorState().getConveyorState());
        io.setConveyorVoltage(0);
      }
    }
  }

  public Command runStateful() {
    return Commands.none();
  }
}

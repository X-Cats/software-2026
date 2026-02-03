package frc.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Conveyor extends SubsystemBase {
  private final ConveyorIO io;
  private final ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();

  public Conveyor(ConveyorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Conveyor", inputs);
  }

  public Command runConveyorMotor() {
    return runEnd(
        () -> {
          io.setConveyorMotorVoltage(ConveyorConstants.CONVEYOR_MOTOR_VOLATGE);
        },
        () -> {
          io.setConveyorMotorVoltage(0.0);
        });
  }
}

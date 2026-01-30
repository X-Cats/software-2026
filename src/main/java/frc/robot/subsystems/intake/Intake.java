package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase implements AutoCloseable{
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command runRollerMotor() {
    return runEnd(
        () -> {
          io.setRollerMotorVoltage(IntakeConstants.ROLLER_MOTOR_VOLTAGE);
        },
        () -> {
          io.setRollerMotorVoltage(0.0);
        });
  }

  public Command stow() {
    return runEnd(
        () -> {
          io.setDeploymentMotorVoltage(IntakeConstants.DEPLOYMENT_MOTOR_VOLTAGE);
        },
        () -> {
          io.setDeploymentMotorVoltage(0.0);
        });
  }

  public Command deploy() {
    return runEnd(
        () -> {
          io.setDeploymentMotorVoltage(-IntakeConstants.DEPLOYMENT_MOTOR_VOLTAGE);
        },
        () -> {
          io.setDeploymentMotorVoltage(0.0);
        });
  }
    
    @Override
    public void close() throws Exception {
        io.close();
    }
}

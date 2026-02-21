package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateMachine;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final RobotStateMachine robotState;

  public Intake(IntakeIO io, RobotStateMachine rs) {
    this.io = io;
    this.robotState = rs;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command runStateful() {
    return new RunCommand(
        () -> {
          if (robotState.getDesiredIntakeState().getExtension()
              == RobotStateMachine.DesiredIntakeState.IntakeExtensionState.EXTENDED) {
            io.setDeploymentMotorVoltage(IntakeConstants.DEPLOYMENT_MOTOR_VOLTAGE);
          } else if (robotState.getDesiredIntakeState().getExtension()
              == RobotStateMachine.DesiredIntakeState.IntakeExtensionState.RETRACTED) {
            io.setDeploymentMotorVoltage(-IntakeConstants.DEPLOYMENT_MOTOR_VOLTAGE);
          } else {
            System.out.println("Unknown Intake Extension State");
            io.setDeploymentMotorVoltage(0);
          }

          if (robotState.getDesiredIntakeState().getRunRoller()) {
            io.setRollerMotorVoltage(IntakeConstants.ROLLER_MOTOR_VOLTAGE);
          } else {
            io.setRollerMotorVoltage(0);
          }
        },
        this);
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
}

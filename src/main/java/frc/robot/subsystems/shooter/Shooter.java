package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final RobotState robotState;

  public Shooter(ShooterIO io, RobotState rs) {
    this.io = io;
    robotState = rs;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public Command runStateful() {
    return new RunCommand(
        () -> {
          if (robotState.getDesiredShooterState().getShooterMode()
              == RobotState.DesiredShooterState.ShooterModeState.ON) {
            io.setShooterMotorVoltage(ShooterConstants.SHOOTER_MOTOR_VOLTAGE);
          } else if (robotState.getDesiredShooterState().getShooterMode()
              == RobotState.DesiredShooterState.ShooterModeState.SUPPRESSED) {
            io.setShooterMotorVoltage(
                ShooterConstants.SHOOTER_MOTOR_VOLTAGE / 2); // NOT REAL, JUST HALF VOLTAGE
          } else if (robotState.getDesiredShooterState().getShooterMode()
              == RobotState.DesiredShooterState.ShooterModeState.OFF) {
            io.setShooterMotorVoltage(0);
          } else {
            System.out.println("Unknown Shooter Mode State");
            // io.setDeploymentMotorVoltage(0);
          }
        },
        this);
  }

  public Command runShooterMotor() {
    return runEnd(
        () -> {
          io.setShooterMotorVoltage(ShooterConstants.SHOOTER_MOTOR_VOLTAGE);
        },
        () -> {
          io.setShooterMotorVoltage(0.0);
        });
  }
}

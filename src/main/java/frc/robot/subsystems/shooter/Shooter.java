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
          switch (robotState.getDesiredShooterState().getShooterMode()) {
            case ON -> io.setShooterMotorVoltage(ShooterConstants.SHOOTER_MOTOR_VOLTAGE);
            case SUPPRESSED -> io.setShooterMotorVoltage(
                ShooterConstants.SHOOTER_MOTOR_VOLTAGE / 2); // NOT REAL, JUST HALF VOLTAGE
            case OFF -> io.setShooterMotorVoltage(0);
            default -> {
              System.out.println(
                  "Illegal Shooter mode : " + robotState.getDesiredShooterState().getShooterMode());
              io.setShooterMotorVoltage(0);
            }
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

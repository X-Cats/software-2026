package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateMachine;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShooterIO.ShooterIOOutputs outputs = new ShooterIO.ShooterIOOutputs();
  private final RobotStateMachine robotState;

  public Shooter(ShooterIO io, RobotStateMachine rs) {
    this.io = io;
    robotState = rs;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    io.applyOutputs(outputs);

    // TODO: not going to look like this, no shooter motor voltages
    switch (robotState.getDesiredShooterState().getShooterMode()) {
      case ON -> io.setShooterMotorRPM(50);
      case IDLE -> io.setShooterMotorRPM(25); // NOT REAL, JUST HALF VOLTAGE
      case OFF -> io.setShooterMotorRPM(0);
      default -> {
        System.out.println(
            "Illegal Shooter mode : " + robotState.getDesiredShooterState().getShooterMode());
        io.setShooterMotorRPM(0);
      }
    }
  }

  public Command runStateful() {
    return Commands.none();
  }

  public Command runShooterMotor() {
    return runEnd(
        () -> {
          io.setShooterMotorRPM(ShooterConstants.SHOOTER_MOTOR_VOLTAGE);
        },
        () -> {
          io.setShooterMotorRPM(0.0);
        });
  }
}

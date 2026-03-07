package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateMachine;
import frc.robot.subsystems.HoodKicker.HoodKickerConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShooterIO.ShooterIOOutputs outputs = new ShooterIO.ShooterIOOutputs();
  private final RobotStateMachine robotState;
  private final LoggedTunableNumber shooterRPM = new LoggedTunableNumber("Shooter/Shoot RPM", 1500);
  private final LoggedTunableNumber shooterCoastRPM =
      new LoggedTunableNumber("Shooter/Coast RPM", 750);


  private static final LoggedTunableNumber kP =
          new LoggedTunableNumber("Shooter/kP", ShooterConstants.kP);
  private static final LoggedTunableNumber kD =
          new LoggedTunableNumber("Shooter/kD", ShooterConstants.kD);

  private final LinearFilter filteredRPM = LinearFilter.singlePoleIIR(0.1, 0.02);
  private double shooterVelocity;

  public Shooter(ShooterIO io, RobotStateMachine rs) {
    this.io = io;
    robotState = rs;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    filteredRPM.calculate(inputs.shooterRPM * 60);
    Logger.processInputs("Shooter", inputs);
    robotState
        .getShooterState()
        .setShooterAtSpeed(
            filteredRPM.lastValue() + 50 > shooterVelocity
                && filteredRPM.lastValue() - 50 < shooterVelocity);

    io.applyOutputs(outputs);

    // TODO: not going to look like this, no shooter motor voltages
    switch (robotState.getDesiredShooterState().getShooterMode()) {
      case ON -> {
        shooterVelocity = shooterRPM.get();
        io.setShooterMotorRPS(shooterVelocity / 60);
      }
      case IDLE -> {
        shooterVelocity = shooterCoastRPM.get();
        io.setShooterMotorRPS(shooterVelocity / 60);
      } // NOT REAL, JUST HALF VOLTAGE
      case OFF -> io.setShooterMotorRPS(0);
      default -> {
        System.out.println(
            "Illegal Shooter mode : " + robotState.getDesiredShooterState().getShooterMode());
        io.setShooterMotorRPS(0);
      }
    }
  }

  public Command runStateful() {
    return Commands.none();
  }

  public Command runShooterMotor() {
    return runEnd(
        () -> {
          io.setShooterMotorRPS(ShooterConstants.SHOOTER_MOTOR_VOLTAGE);
        },
        () -> {
          io.setShooterMotorRPS(0.0);
        });
  }
}

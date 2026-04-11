package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateMachine;
import frc.robot.util.LaunchCalculator;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShooterIO.ShooterIOOutputs outputs = new ShooterIO.ShooterIOOutputs();
  private final RobotStateMachine robotState;
  private final LoggedTunableNumber shooterRPM = new LoggedTunableNumber("Shooter/Shoot RPM", 2000);
  private final LoggedTunableNumber shooterCoastRPM =
      new LoggedTunableNumber("Shooter/Coast RPM", 750);

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter/kP", ShooterConstants.kP);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Shooter/kD", ShooterConstants.kD);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Shooter/kV", ShooterConstants.kV);
  private final LinearFilter filteredRPM = LinearFilter.singlePoleIIR(0.1, 0.02);

  private final String logKey;

  public Shooter(ShooterIO io, RobotStateMachine rs) {
    this.io = io;
    this.logKey = "Shooter/" + io.getShooterSide();
    robotState = rs;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    double currentRPM;
    currentRPM = filteredRPM.calculate(inputs.shooterRPM);
    //    filteredRPM.calculate(inputs.shooterRPM);
    Logger.processInputs(logKey, inputs);

    outputs.kP = kP.getAsDouble();
    outputs.kD = kD.getAsDouble();
    outputs.kV = kV.getAsDouble();

    switch (robotState.getDesiredShooterState().getShooterMode()) {
      case ON -> {
        outputs.velocityRPM = LaunchCalculator.getInstance().getParameters().flywheelSpeed();
      }
      case IDLE -> {
        outputs.velocityRPM = shooterCoastRPM.get();
      }
      case OFF -> outputs.velocityRPM = 0;
      default -> {
        System.out.println(
            "Illegal Shooter mode : " + robotState.getDesiredShooterState().getShooterMode());
        io.setShooterMotorVoltage(0);
      }
    }
    robotState
        .getShooterState()
        .setShooterAtSpeed(
            filteredRPM.lastValue() + 100 > outputs.velocityRPM
                && filteredRPM.lastValue() - 100 < outputs.velocityRPM);
    if (outputs.velocityRPM < currentRPM - 100) { // TODO: apply a deadband of some sort here
      outputs.idleDown = true;
    } else {
      outputs.idleDown = false;
    }
    io.applyOutputs(outputs);
  }
}

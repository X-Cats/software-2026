package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateMachine;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final RobotStateMachine robotState;

  private LinearFilter deployVelocityFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  private LinearFilter deployPositionFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

  private LoggedTunableNumber intakeDeployTorque =
      new LoggedTunableNumber("Intake/Deploy Amps", IntakeConstants.DEPLOYMENT_MOTOR_CURRENT);
  private LoggedTunableNumber intakeStowTorque =
      new LoggedTunableNumber("Intake/Stow Amps", IntakeConstants.DEPLOYMENT_MOTOR_STOW_CURRENT);

  private LoggedTunableNumber intakeRollerAmps =
      new LoggedTunableNumber("Intake/Roller Amps", IntakeConstants.ROLLER_MOTOR_TORQUE);

  public Intake(IntakeIO io, RobotStateMachine rs) {
    this.io = io;
    this.robotState = rs;
    io.zeroDeploy();
  }

  private boolean shouldRunRoller() {
    return (inputs.deployIn == 0) && deployPositionFilter.lastValue() > 1;
  }

  private boolean deployIsStopped() {
    return deployVelocityFilter.lastValue() <= 5 && deployVelocityFilter.lastValue() >= -5;
  }

  @Override
  public void periodic() {
    deployVelocityFilter.calculate(inputs.deployVelocity);
    deployPositionFilter.calculate(inputs.deployPosition);
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    switch (robotState.getDesiredIntakeState().getDesiredIntakeDeployState()) {
      case DEPLOYED -> {
        runOut();
      }
      case STOWED -> {
        runIn();
      }
      case AGITATING -> {
        if (((int) (Timer.getFPGATimestamp() / 10)) % 3 == 0) {
          runOut();
        } else {
          runIn();
        }
      }
      default -> {
        System.out.println("Unknown Intake Extension State");
        io.setDeployMotorTorque(0);
      }
    }

    /*
    TODO: Only run the roller if we're out and not running
     */
    if (robotState.getDesiredIntakeState().getDesiredIntakeRollerState()
            == RobotStateMachine.DesiredIntakeState.IntakeRollerState.INTAKING
        && shouldRunRoller()) {
      io.setRollerMotorTorque(intakeRollerAmps.getAsDouble());
    } else {
      io.setRollerMotorTorque(0);
    }
  }

  private void runIn() {
    if (inputs.deployIn == 0) {
      io.setDeployMotorTorque(-intakeStowTorque.getAsDouble());
    } else {
      io.setDeployMotorTorque(0);
    }
  }

  private void runOut() {
    if (inputs.deployOut == 0) {
      io.setDeployMotorTorque(intakeDeployTorque.getAsDouble());
    } else {
      io.setDeployMotorTorque(0);
    }
  }
}

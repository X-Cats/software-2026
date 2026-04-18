package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotStateMachine;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIO.IntakeIOTunables tunables = new IntakeIO.IntakeIOTunables();
  private final RobotStateMachine robotState;

  private LinearFilter deployVelocityFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  private LinearFilter deployPositionFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

  private LoggedTunableNumber intakeDeployTorque =
      new LoggedTunableNumber("Intake/Deploy Amps", IntakeConstants.DEPLOYMENT_MOTOR_CURRENT);
  private LoggedTunableNumber intakeStowTorque =
      new LoggedTunableNumber("Intake/Stow Amps", IntakeConstants.DEPLOYMENT_MOTOR_STOW_CURRENT);
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Intake/Deploy/kP", IntakeConstants.DEPLOYMENT_KP);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Intake/Deploy/kD", IntakeConstants.DEPLOYMENT_KD);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Intake/Deploy/kV", IntakeConstants.DEPLOYMENT_KV);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Intake/Deploy/kI", IntakeConstants.DEPLOYMENT_KI);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Intake/Deploy/kS", IntakeConstants.DEPLOYMENT_KS);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Intake/Deploy/kA", IntakeConstants.DEPLOYMENT_KA);
  private static final LoggedTunableNumber deployMMCruiseVelocity =
      new LoggedTunableNumber(
          "Intake/Deploy/Motion Magic/Cruise Velocity",
          IntakeConstants.DEPLOYMENT_MM_CRUISE_VELOCITY);
  private static final LoggedTunableNumber deployMMAcceleration =
      new LoggedTunableNumber(
          "Intake/Deploy/Motion Magic/Acceleration",
          IntakeConstants.DEPLOYMENT_MM_CRUISE_ACCELERATION);
  private static final LoggedTunableNumber deployMMJerk =
      new LoggedTunableNumber(
          "Intake/Deploy/Motion Magic/Jerk", IntakeConstants.DEPLOYMENT_MM_CRUISE_JERK);

  private LoggedTunableNumber intakeRollerAmps =
      new LoggedTunableNumber("Intake/Roller Amps", IntakeConstants.ROLLER_MOTOR_TORQUE);

  public Intake(IntakeIO io, RobotStateMachine rs) {
    this.io = io;
    this.robotState = rs;
    io.zeroDeploy();
  }

  private boolean shouldRunRoller() {
    return (inputs.deployIn == 0) && deployPositionFilter.lastValue() > 0.5;
  }

  private boolean deployIsStopped() {
    return deployVelocityFilter.lastValue() <= 5 && deployVelocityFilter.lastValue() >= -5;
  }

  private double agitateSetpoint = IntakeConstants.DEPLOY_AGITATE_CLOSE;

  @Override
  public void periodic() {
    deployVelocityFilter.calculate(inputs.deployVelocity);
    deployPositionFilter.calculate(inputs.deployPosition);
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (Constants.tuningMode) {
      updateTunables();
      LoggedTunableNumber.ifChanged(
          1,
          () -> io.applyTunables(tunables),
          kS,
          kV,
          kA,
          kP,
          kI,
          kD,
          deployMMCruiseVelocity,
          deployMMAcceleration,
          deployMMJerk);
    }
    switch (robotState.getDesiredIntakeState().getDesiredIntakeDeployState()) {
      case DEPLOYED -> {
        runOut();
      }
      case STOWED -> {
        runIn();
      }
      case AGITATING -> {
        if (inputs.deployAtSetpoint
            == 1) { // Flip-flop; wait until deploy at setpoint before making another change
          if (inputs.deploySetpoint <= IntakeConstants.DEPLOY_AGITATE_FAR)
            agitateSetpoint = IntakeConstants.DEPLOY_AGITATE_FAR;
          else agitateSetpoint = IntakeConstants.DEPLOY_AGITATE_CLOSE;
        }
        io.setDeployMotorPosition(agitateSetpoint);
      }
      default -> {
        System.out.println("Unknown Intake Extension State");
        io.setDeployMotorTorque(0);
      }
    }

    if (shouldRunRoller()) {
      switch (robotState.getDesiredIntakeState().getDesiredIntakeRollerState()) {
        case INTAKING -> {
          if (inputs.deployPosition > 15) {
            io.setRollerMotorSpeed(IntakeConstants.ROLLER_MOTOR_VELOCITY);
          } else {
            io.setRollerMotorVoltage(0);
          }
        }
        case EJECTING -> {
          // Do nothing
        }
        case AGITATING -> {
          if (((int) (Timer.getFPGATimestamp() * 10)) % 3
              == 0) { // Every 1/3 of the time we agitate
            io.setRollerMotorTorque(IntakeConstants.ROLLER_MOTOR_TORQUE / 2);
          } else {
            io.setRollerMotorTorque(-IntakeConstants.ROLLER_MOTOR_TORQUE / 3);
          }
        }
        case OFF -> {
          io.setRollerMotorTorque(0);
        }
      }
    } else {
      io.setRollerMotorTorque(0);
    }
  }

  public void updateTunables() {
    tunables.deploymentKS = kS.getAsDouble();
    tunables.deploymentKV = kV.getAsDouble();
    tunables.deploymentKA = kA.getAsDouble();
    tunables.deploymentKP = kP.getAsDouble();
    tunables.deploymentKI = kI.getAsDouble();
    tunables.deploymentKD = kD.getAsDouble();
    tunables.deploymentMMCruiseVelocity = deployMMCruiseVelocity.getAsDouble();
    tunables.deploymentMMAcceleration = deployMMAcceleration.getAsDouble();
    tunables.deploymentMMJerk = deployMMJerk.getAsDouble();
  }

  private void zero() {
    io.setDeployMotorTorque(-2);
  }

  private void runIn() {
    io.setDeployMotorPosition(IntakeConstants.DEPLOYMENT_STOWED_SETPOINT);
  }

  private void runOut() {
    io.setDeployMotorPosition(IntakeConstants.DEPLOYMENT_DEPLOYED_SETPOINT);
  }

  // TODO: add FF here?
  private void agitateIn() {
    runIn();
  }

  private void agitateOut() {
    runOut();
  }
}

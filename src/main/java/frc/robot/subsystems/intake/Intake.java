package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateMachine;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final RobotStateMachine robotState;

  private LinearFilter deployVelocityFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  private LinearFilter deployPositionFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

  public Intake(IntakeIO io, RobotStateMachine rs) {
    this.io = io;
    this.robotState = rs;
    io.zeroDeploy();
  }

  private boolean shouldRunRoller() {
    return robotState.getDesiredIntakeState().getDesiredIntakeRollerState()
            == RobotStateMachine.DesiredIntakeState.IntakeRollerState.INTAKING
        && deployIsStopped();
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

    if (robotState.getDesiredIntakeState().getDesiredIntakeDeployState()
        == RobotStateMachine.DesiredIntakeState.IntakeDeployState.DEPLOYED) {
      io.setDeployMotorTorque(IntakeConstants.DEPLOYMENT_MOTOR_CURRENT);
    } else if (robotState.getDesiredIntakeState().getDesiredIntakeDeployState()
        == RobotStateMachine.DesiredIntakeState.IntakeDeployState.STOWED) {
      if (!(deployPositionFilter.lastValue() < 1 && deployIsStopped())) {
        io.setDeployMotorTorque(-IntakeConstants.DEPLOYMENT_MOTOR_STOW_CURRENT);
      }
    } else {
      System.out.println("Unknown Intake Extension State");
      io.setDeployMotorTorque(0);
    }

    /*
    TODO: Only run the roller if we're out and not running
     */
    if (this.shouldRunRoller()) {
      io.setRollerMotorTorque(IntakeConstants.ROLLER_MOTOR_TORQUE);
    } else {
      io.setRollerMotorTorque(0);
    }
  }
}

package frc.robot.subsystems.intake;

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

    if (robotState.getDesiredIntakeState().getDesiredIntakeDeployState()
        == RobotStateMachine.DesiredIntakeState.IntakeDeployState.DEPLOYED) {
      io.setDeployMotorTorque(IntakeConstants.DEPLOYMENT_MOTOR_CURRENT);
    } else if (robotState.getDesiredIntakeState().getDesiredIntakeDeployState()
        == RobotStateMachine.DesiredIntakeState.IntakeDeployState.STOWED) {
      io.setDeployMotorTorque(-IntakeConstants.DEPLOYMENT_MOTOR_STOW_CURRENT);
    } else {
      System.out.println("Unknown Intake Extension State");
      io.setDeployMotorTorque(0);
    }

    if (robotState.getDesiredIntakeState().getDesiredIntakeRollerState()
        == RobotStateMachine.DesiredIntakeState.IntakeRollerState.INTAKING) {
      io.setRollerMotorTorque(IntakeConstants.ROLLER_MOTOR_TORQUE);
    } else {
      io.setRollerMotorTorque(0);
    }
  }
}

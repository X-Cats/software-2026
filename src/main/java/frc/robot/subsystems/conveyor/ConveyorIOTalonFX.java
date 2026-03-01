package frc.robot.subsystems.conveyor;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;

public class ConveyorIOTalonFX implements ConveyorIO {
  private final TalonFX conveyorLeader = new TalonFX(ConveyorConstants.CONVEYOR_MOTOR_ID);
  private final TalonFX conveyorFollower = new TalonFX(ConveyorConstants.CONVEYOR_FOLLOWER_ID);
  private final StatusSignal<Voltage> conveyorAppliedVolts = conveyorLeader.getMotorVoltage();

  private final TorqueCurrentFOC conveyorTorqueRequest =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  public ConveyorIOTalonFX() {
    var conveyorConfig = new TalonFXConfiguration();
    conveyorConfig.CurrentLimits.SupplyCurrentLimit =
        ConveyorConstants.CONVEYOR_MOTOR_CURRENT_LIMIT;
    conveyorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    conveyorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    conveyorLeader.getConfigurator().apply(conveyorConfig, 0.25);

    var followerConfig = conveyorConfig.clone();
    // followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> conveyorFollower.getConfigurator().apply(followerConfig, 0.25));

    conveyorFollower.setControl(
        new Follower(conveyorLeader.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void setConveyorTorque(double amps) {
    // conveyorLeader.setControl(conveyorTorqueRequest.withOutput(amps));
  }
}

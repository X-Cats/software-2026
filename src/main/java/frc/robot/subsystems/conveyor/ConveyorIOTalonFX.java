package frc.robot.subsystems.conveyor;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil;

public class ConveyorIOTalonFX implements ConveyorIO {
  private final TalonFX conveyorLeader = new TalonFX(ConveyorConstants.CONVEYOR_MOTOR_ID);
  private final TalonFX conveyorFollower = new TalonFX(ConveyorConstants.CONVEYOR_FOLLOWER_ID);
  private final StatusSignal<Voltage> conveyorAppliedVolts = conveyorLeader.getMotorVoltage();
  private final StatusSignal<Current> conveyorSupplyCurrent = conveyorLeader.getSupplyCurrent();
  private final StatusSignal<Current> conveyorStatorCurrent = conveyorLeader.getStatorCurrent();

  private LoggedTunableNumber conveyorSupplyCurrentLimit =
      new LoggedTunableNumber(
          "Conveyor/Supply Current", ConveyorConstants.CONVEYOR_MOTOR_CURRENT_LIMIT);
  private LoggedTunableNumber conveyorTorqueCurrentLimit =
      new LoggedTunableNumber(
          "Conveyor/Torque Current", ConveyorConstants.CONVEYOR_MOTOR_TORQUE_LIMIT);

  private final TorqueCurrentFOC conveyorTorqueRequest =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut conveyorVoltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);
  private Integer monotonic = 0;

  private final TalonFXConfiguration conveyorConfig = new TalonFXConfiguration();

  public ConveyorIOTalonFX() {
    conveyorConfig.CurrentLimits.SupplyCurrentLimit =
        ConveyorConstants.CONVEYOR_MOTOR_CURRENT_LIMIT;
    conveyorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    conveyorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    conveyorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    conveyorLeader.getConfigurator().apply(conveyorConfig, 0.25);

    var followerConfig = conveyorConfig.clone();
    // followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> conveyorFollower.getConfigurator().apply(followerConfig, 0.25));

    conveyorFollower.setControl(
        new Follower(conveyorLeader.getDeviceID(), MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, conveyorSupplyCurrent, conveyorStatorCurrent, conveyorAppliedVolts);

    PhoenixUtil.registerSignals(
        false, conveyorAppliedVolts, conveyorStatorCurrent, conveyorSupplyCurrent);
  }

  public void applyOutputs() {
    if (Constants.tuningMode) {
      if (conveyorSupplyCurrentLimit.hasChanged(conveyorSupplyCurrentLimit.hashCode())
          || conveyorTorqueCurrentLimit.hasChanged(conveyorTorqueCurrentLimit.hashCode())) {
        monotonic++;
        conveyorConfig.CurrentLimits.SupplyCurrentLimit = conveyorSupplyCurrentLimit.get();
        conveyorConfig.CurrentLimits.StatorCurrentLimit = conveyorTorqueCurrentLimit.get();

        conveyorLeader.getConfigurator().apply(conveyorConfig);
        conveyorFollower.getConfigurator().apply(conveyorConfig);
      }
    }
  }

  public void updateInputs(ConveyorIOInputs inputs) {
    inputs.conveyorAppliedVolts = conveyorAppliedVolts.getValueAsDouble();
    inputs.conveyorSupplyCurrent = conveyorSupplyCurrent.getValueAsDouble();
    inputs.conveyorStatorCurrent = conveyorStatorCurrent.getValueAsDouble();
  }

  @Override
  public void setConveyorVoltage(double volts) {
    conveyorLeader.setControl(conveyorVoltageRequest.withOutput(volts));
  }
}

package frc.robot.subsystems.HoodKicker;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class HoodKickerIOTalonFX implements HoodKickerIO {
  // Motors
  private final TalonFX hood = new TalonFX(HoodKickerConstants.HOOD_MOTOR_ID);
  private final TalonFX kicker = new TalonFX(HoodKickerConstants.KICKER_MOTOR_ID);
  private final TalonFX kickerFollower = new TalonFX(HoodKickerConstants.KICKER_MOTOR_FOLLOWER_ID);
  private final CANdi hoodLimits = new CANdi(HoodKickerConstants.CANDI_CAN_ID);

  // Control Requests
  private final PositionTorqueCurrentFOC positionControl =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  private final Slot0Configs hoodSlot0 = new Slot0Configs();

  private final VoltageOut zeroControl = new VoltageOut(-2).withUpdateFreqHz(0.0);

  // Loggables
  private final StatusSignal<Voltage> hoodAppliedVolts = hood.getMotorVoltage();
  private final StatusSignal<AngularVelocity> hoodVelocity = hood.getVelocity();
  private final StatusSignal<Angle> hoodPosition = hood.getPosition();
  private final StatusSignal<Current> hoodTorqueCurrent = hood.getTorqueCurrent();
  private final StatusSignal<Current> hoodSupplyCurrent = hood.getSupplyCurrent();
  private final StatusSignal<Boolean> hoodForwardLimit = hood.getFault_ForwardHardLimit();
  private final StatusSignal<Boolean> hoodReverseLimit = hood.getFault_ReverseHardLimit();

  private final StatusSignal<Voltage> kickerAppliedVolts = kicker.getMotorVoltage();
  private final StatusSignal<AngularVelocity> kickerVelocity = kicker.getVelocity();
  private final StatusSignal<Current> kickerTorqueCurrent = kicker.getTorqueCurrent();
  private final StatusSignal<Current> kickerSupplyCurrent = kicker.getSupplyCurrent();

  public HoodKickerIOTalonFX() {
    var hoodLimitsConfig = new CANdiConfiguration();
    hoodLimitsConfig.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenLow;
    hoodLimitsConfig.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenLow;
    hoodLimitsConfig.DigitalInputs.S1FloatState = S1FloatStateValue.FloatDetect;
    hoodLimitsConfig.DigitalInputs.S2FloatState = S2FloatStateValue.FloatDetect;
    tryUntilOk(5, () -> hoodLimits.getConfigurator().apply(hoodLimitsConfig));

    var hoodConfig = new TalonFXConfiguration();
    hoodConfig.CurrentLimits.SupplyCurrentLimit = HoodKickerConstants.HOOD_MOTOR_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.Feedback.SensorToMechanismRatio = HoodKickerConstants.HOOD_MOTOR_REDUCTION;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hoodConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANdiS1;
    hoodConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS2;
    hoodConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    hoodConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
    hoodConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    hoodConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;
    hoodConfig.HardwareLimitSwitch.ForwardLimitRemoteSensorID = HoodKickerConstants.CANDI_CAN_ID;
    hoodConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = HoodKickerConstants.CANDI_CAN_ID;
    tryUntilOk(5, () -> hood.getConfigurator().apply(hoodConfig, 0.25));

    hoodSlot0.kP = HoodKickerConstants.kP;
    hoodSlot0.kI = HoodKickerConstants.kI;
    hoodSlot0.kD = HoodKickerConstants.kD;
    hoodSlot0.kS = HoodKickerConstants.kS;
    hoodSlot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    hood.getConfigurator().apply(hoodSlot0);

    var kickerConfig = new TalonFXConfiguration();
    kickerConfig.CurrentLimits.SupplyCurrentLimit = HoodKickerConstants.KICKER_MOTOR_CURRENT_LIMIT;
    kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kickerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> kicker.getConfigurator().apply(kickerConfig, 0.25));
    tryUntilOk(5, () -> kickerFollower.getConfigurator().apply(kickerConfig, 0.25));
    kickerFollower.setControl(new Follower(kicker.getDeviceID(), MotorAlignmentValue.Aligned));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        hoodAppliedVolts,
        hoodVelocity,
        hoodPosition,
        hoodTorqueCurrent,
        hoodSupplyCurrent,
        hoodForwardLimit,
        hoodReverseLimit,
        kickerAppliedVolts,
        kickerVelocity,
        kickerSupplyCurrent,
        kickerTorqueCurrent);

    kicker.optimizeBusUtilization();
    kickerFollower.optimizeBusUtilization();
    hood.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        false,
        hoodAppliedVolts,
        hoodVelocity,
        hoodPosition,
        hoodTorqueCurrent,
        hoodSupplyCurrent,
        hoodForwardLimit,
        hoodReverseLimit,
        kickerAppliedVolts,
        kickerVelocity,
        kickerSupplyCurrent,
        kickerTorqueCurrent);
  }

  public void updateInputs(HoodKickerIO.HoodIOInputs inputs) {
    // Hood
    inputs.hoodAppliedVolts = hoodAppliedVolts.getValueAsDouble();
    inputs.hoodVelocity = hoodVelocity.getValueAsDouble();
    inputs.hoodPosition = hoodPosition.getValueAsDouble();
    inputs.hoodTorqueCurrent = hoodTorqueCurrent.getValueAsDouble();
    inputs.hoodSupplyCurrent = hoodSupplyCurrent.getValueAsDouble();
    inputs.hoodForwardLimit = hoodForwardLimit.getValueAsDouble();
    inputs.hoodReverseLimit = hoodReverseLimit.getValueAsDouble();

    // Kicker
    inputs.kickerAppliedVolts = kickerAppliedVolts.getValueAsDouble();
    inputs.kickerVelocity = kickerVelocity.getValueAsDouble();
    inputs.kickerTorqueCurrent = kickerTorqueCurrent.getValueAsDouble();
    inputs.kickerSupplyCurrent = kickerSupplyCurrent.getValueAsDouble();
  }

  public void setHoodPosition(double radians) {
    hood.setControl(positionControl.withPosition(radians));
  }

  public void setKickerMotorVoltage(double volts) {
    kicker.setVoltage(volts);
  }

  public void zero() {
    hood.setControl(zeroControl);
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    hood.setControl(
        positionControl.withPosition(outputs.positionRad).withSlot(0).withFeedForward(outputs.kS));
  }

  public void applyTunables(HoodIOOutputs outputs) {
    if (Constants.tuningMode) {
      var configs = hoodSlot0;
      configs.kP = outputs.kP;
      configs.kD = outputs.kD;
      configs.kS = outputs.kS;
      tryUntilOk(5, () -> hood.getConfigurator().apply(configs));
    }
  }
}

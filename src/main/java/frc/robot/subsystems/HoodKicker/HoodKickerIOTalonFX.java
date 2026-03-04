package frc.robot.subsystems.HoodKicker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.PhoenixUtil;

public class HoodKickerIOTalonFX implements HoodKickerIO {
  // Motors
  private final TalonFX hood = new TalonFX(HoodKickerConstants.HOOD_MOTOR_ID);
  private final TalonFX kicker = new TalonFX(HoodKickerConstants.KICKER_MOTOR_ID);

  // Control Requests
  private final PositionTorqueCurrentFOC positionControl =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  // Loggables
  private final StatusSignal<Voltage> hoodAppliedVolts = hood.getMotorVoltage();
  private final StatusSignal<AngularVelocity> hoodVelocity = hood.getVelocity();
  private final StatusSignal<Angle> hoodPosition = hood.getPosition();
  private final StatusSignal<Current> hoodTorqueCurrent = hood.getTorqueCurrent();
  private final StatusSignal<Current> hoodSupplyCurrent = hood.getSupplyCurrent();

  private final StatusSignal<Voltage> kickerAppliedVolts = kicker.getMotorVoltage();
  private final StatusSignal<AngularVelocity> kickerVelocity = kicker.getVelocity();
  private final StatusSignal<Current> kickerTorqueCurrent = kicker.getTorqueCurrent();
  private final StatusSignal<Current> kickerSupplyCurrent = kicker.getSupplyCurrent();

  public HoodKickerIOTalonFX() {
    var hoodConfig = new TalonFXConfiguration();
    hoodConfig.CurrentLimits.SupplyCurrentLimit = HoodKickerConstants.HOOD_MOTOR_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hood.getConfigurator().apply(hoodConfig, 0.25);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = ShooterConstants.kP;
    slot0Configs.kI = ShooterConstants.kI;
    slot0Configs.kD = ShooterConstants.kD;
    hood.getConfigurator().apply(slot0Configs);

    var kickerConfig = new TalonFXConfiguration();
    kickerConfig.CurrentLimits.SupplyCurrentLimit = HoodKickerConstants.KICKER_MOTOR_CURRENT_LIMIT;
    kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kickerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    kicker.getConfigurator().apply(kickerConfig, 0.25);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        hoodAppliedVolts,
        hoodVelocity,
        hoodPosition,
        hoodTorqueCurrent,
        hoodSupplyCurrent,
        kickerAppliedVolts,
        kickerVelocity,
        kickerSupplyCurrent,
        kickerTorqueCurrent);

    kicker.optimizeBusUtilization();
    hood.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        false,
        hoodAppliedVolts,
        hoodVelocity,
        hoodPosition,
        hoodTorqueCurrent,
        hoodSupplyCurrent,
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

    // Kicker
    inputs.kickerAppliedVolts = kickerAppliedVolts.getValueAsDouble();
    inputs.kickerVelocity = kickerVelocity.getValueAsDouble();
    inputs.kickerTorqueCurrent = kickerTorqueCurrent.getValueAsDouble();
    inputs.kickerSupplyCurrent = kickerSupplyCurrent.getValueAsDouble();
  }

  public void setHoodPosition(double ticks) {
    hood.setControl(positionControl.withPosition(ticks));
  }

  public void setKickerMotorVoltage(double volts) {
    kicker.setVoltage(volts);
  }
}

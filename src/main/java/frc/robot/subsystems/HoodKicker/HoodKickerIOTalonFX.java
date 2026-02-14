package frc.robot.subsystems.HoodKicker;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;

public class HoodKickerIOTalonFX implements HoodKickerIO {
  private final TalonFX hood = new TalonFX(HoodKickerConstants.HOOD_MOTOR_ID);
  private final StatusSignal<Voltage> hoodAppliedVolts = hood.getMotorVoltage();
  private final TalonFX kicker = new TalonFX(HoodKickerConstants.KICKER_MOTOR_ID);
  private final StatusSignal<Voltage> kickerAppliedVolts = kicker.getMotorVoltage();

  public HoodKickerIOTalonFX() {
    var hoodConfig = new TalonFXConfiguration();
    hoodConfig.CurrentLimits.SupplyCurrentLimit = HoodKickerConstants.HOOD_MOTOR_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hood.getConfigurator().apply(hoodConfig, 0.25);

    var kickerConfig = new TalonFXConfiguration();
    kickerConfig.CurrentLimits.SupplyCurrentLimit = HoodKickerConstants.KICKER_MOTOR_CURRENT_LIMIT;
    kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kicker.getConfigurator().apply(kickerConfig, 0.25);
  }
}

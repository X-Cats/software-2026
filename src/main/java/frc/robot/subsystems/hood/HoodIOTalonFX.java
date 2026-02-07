package frc.robot.subsystems.hood;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX hood = new TalonFX(HoodConstants.HOOD_MOTOR_ID);
  private final StatusSignal<Voltage> hoodAppliedVolts = hood.getMotorVoltage();

  public HoodIOTalonFX() {
    var hoodConfig = new TalonFXConfiguration();
    hoodConfig.CurrentLimits.SupplyCurrentLimit = HoodConstants.HOOD_MOTOR_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hood.getConfigurator().apply(hoodConfig, 0.25);
  }
}

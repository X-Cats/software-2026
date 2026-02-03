package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;

public class HopperIOTalonFX implements HopperIO {
  private final TalonFX hopper = new TalonFX(HopperConstants.HOPPER_MOTOR_ID);
  private final StatusSignal<Voltage> hopperAppliedVolts = hopper.getMotorVoltage();

  public HopperIOTalonFX() {
    var hopperConfig = new TalonFXConfiguration();
    hopperConfig.CurrentLimits.SupplyCurrentLimit = HopperConstants.HOPPER_MOTOR_ID;
    hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hopper.getConfigurator().apply(hopperConfig, 0.25);
    //        tryUntilOk(5, ()->  hopper.getConfigurator().apply(hopperConfig, 0.25));

  }
}

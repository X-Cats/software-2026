package frc.robot.subsystems.HoodKicker;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterConstants;

public class HoodKickerIOTalonFX implements HoodKickerIO {
  private final TalonFX hood;
  private final StatusSignal<Voltage> hoodAppliedVolts;
  private final TalonFX kicker = new TalonFX(HoodKickerConstants.KICKER_MOTOR_ID);
  private final StatusSignal<Voltage> kickerAppliedVolts = kicker.getMotorVoltage();

  private final PositionTorqueCurrentFOC positionControl =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  public HoodKickerIOTalonFX() {

    hood = new TalonFX(HoodKickerConstants.HOOD_MOTOR_ID, CANBus.roboRIO());
    hoodAppliedVolts = hood.getMotorVoltage();

    var hoodConfig = new TalonFXConfiguration();
    hoodConfig.CurrentLimits.SupplyCurrentLimit = HoodKickerConstants.HOOD_MOTOR_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hood.getConfigurator().apply(hoodConfig, 0.25);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = ShooterConstants.kS;
    slot0Configs.kV = ShooterConstants.kV;
    slot0Configs.kP = ShooterConstants.kP;
    slot0Configs.kI = ShooterConstants.kI;
    slot0Configs.kD = ShooterConstants.kD;

    hood.getConfigurator().apply(slot0Configs);

    var kickerConfig = new TalonFXConfiguration();
    kickerConfig.CurrentLimits.SupplyCurrentLimit = HoodKickerConstants.KICKER_MOTOR_CURRENT_LIMIT;
    kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kicker.getConfigurator().apply(kickerConfig, 0.25);

    // System.out.println(hood.getDescription());

  }

  public void updateInputs(HoodKickerIO.HoodIOInputs inputs) {
    inputs.hoodAppliedVolts = hoodAppliedVolts.getValueAsDouble();
    inputs.kickerAppliedVolts = kicker.getMotorVoltage().getValueAsDouble();
  }

  public void setHoodPosition(double ticks) {
    hood.setControl(positionControl.withPosition(Units.radiansToRotations(ticks)));
  }
}

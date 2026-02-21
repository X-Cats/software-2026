package frc.robot.subsystems.conveyor;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;

public class ConveyorIOTalonFX implements ConveyorIO {
  private final TalonFX conveyor = new TalonFX(ConveyorConstants.CONVEYOR_MOTOR_ID);
  private final StatusSignal<Voltage> conveyorAppliedVolts = conveyor.getMotorVoltage();

  public ConveyorIOTalonFX() {
    var conveyorConfig = new TalonFXConfiguration();
    conveyorConfig.CurrentLimits.SupplyCurrentLimit =
        ConveyorConstants.CONVEYOR_MOTOR_CURRENT_LIMIT;
    conveyorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    conveyorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    conveyor.getConfigurator().apply(conveyorConfig, 0.25);
    //        tryUntilOk(5, ()->  hopper.getConfigurator().apply(hopperConfig, 0.25));

  }
}

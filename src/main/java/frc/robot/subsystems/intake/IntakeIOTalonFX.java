package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX roller = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);
  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Current> rollerTorqueCurrent;
  private final StatusSignal<Current> rollerSupplyCurrent;

  private final TorqueCurrentFOC rollerTorqueCurrentRequest = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC deployTorqueCurrentRequest = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  private final TalonFX deploy = new TalonFX(IntakeConstants.DEPLOYMENT_MOTOR_ID);
  private final StatusSignal<Voltage> deployAppliedVolts = deploy.getMotorVoltage();

  public IntakeIOTalonFX() {
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_MOTOR_CURRENT_LIMIT;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tryUntilOk(5, ()->  roller.getConfigurator().apply(rollerConfig, 0.25));
    rollerAppliedVolts = roller.getMotorVoltage();
    rollerVelocity = roller.getVelocity();
    rollerTorqueCurrent = roller.getTorqueCurrent();
    rollerSupplyCurrent = roller.getSupplyCurrent();


    var deployConfig = new TalonFXConfiguration();
    deployConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.DEPLOYMENT_MOTOR_CURRENT_LIMIT;
    deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    deploy.getConfigurator().apply(deployConfig, 0.25);
    //        tryUntilOk(5, ()->  deploy.getConfigurator().apply(deployConfig, 0.25));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerVelocity = rollerVelocity.getValueAsDouble();
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerTorqueCurrentAmps = rollerTorqueCurrent.getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = rollerSupplyCurrent.getValueAsDouble();
  }

  @Override
  public void setRollerMotorTorque(double torque) {
    roller.setControl(rollerTorqueCurrentRequest.withOutput(torque));
  }

  @Override
  public void setDeployMotorTorque(double amps) {
    deploy.setControl(deployTorqueCurrentRequest.withOutput(amps));
  }

}

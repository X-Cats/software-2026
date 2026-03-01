package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class IntakeIOTalonFX implements IntakeIO {
  // Motors
  private final TalonFX roller = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);
  private final TalonFX deploy = new TalonFX(IntakeConstants.DEPLOYMENT_MOTOR_ID);

  // Control Requests
  private final TorqueCurrentFOC rollerTorqueCurrentRequest =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC deployTorqueCurrentRequest =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  // LOGGABLES
  private final StatusSignal<Voltage> deployAppliedVolts = deploy.getMotorVoltage();
  private final StatusSignal<AngularVelocity> deployVelocity = deploy.getVelocity();
  private final StatusSignal<Angle> deployPosition = deploy.getPosition();
  private final StatusSignal<Current> deployTorqueCurrent = deploy.getTorqueCurrent();
  private final StatusSignal<Current> deploySupplyCurrent = deploy.getSupplyCurrent();

  private final StatusSignal<Voltage> rollerAppliedVolts = roller.getMotorVoltage();
  private final StatusSignal<AngularVelocity> rollerVelocity = roller.getVelocity();
  private final StatusSignal<Current> rollerTorqueCurrent = roller.getTorqueCurrent();
  private final StatusSignal<Current> rollerSupplyCurrent = roller.getSupplyCurrent();

  public IntakeIOTalonFX() {
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_MOTOR_CURRENT_LIMIT;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> roller.getConfigurator().apply(rollerConfig, 0.25));

    var deployConfig = new TalonFXConfiguration();
    deployConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.DEPLOYMENT_MOTOR_CURRENT_LIMIT;
    deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    deploy.getConfigurator().apply(deployConfig, 0.25);
    tryUntilOk(5, () -> deploy.getConfigurator().apply(deployConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        rollerVelocity,
        rollerAppliedVolts,
        rollerTorqueCurrent,
        rollerSupplyCurrent,
        deployVelocity,
        deployPosition,
        deployAppliedVolts,
        deployTorqueCurrent,
        deploySupplyCurrent);

    roller.optimizeBusUtilization();
    deploy.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        false,
        rollerVelocity,
        rollerAppliedVolts,
        rollerTorqueCurrent,
        rollerSupplyCurrent,
        deployAppliedVolts,
        deployVelocity,
        deployPosition,
        deployTorqueCurrent,
        deploySupplyCurrent);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerVelocity = rollerVelocity.getValueAsDouble();
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerTorqueCurrentAmps = rollerTorqueCurrent.getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = rollerSupplyCurrent.getValueAsDouble();

    inputs.deployAppliedVolts = deployAppliedVolts.getValueAsDouble();
    inputs.deployVelocity = deployVelocity.getValueAsDouble();
    inputs.deployPosition = deployPosition.getValueAsDouble();
    inputs.deployTorqueCurrentAmps = deployTorqueCurrent.getValueAsDouble();
    inputs.deploySupplyCurrentAmps = deploySupplyCurrent.getValueAsDouble();
  }

  @Override
  public void setRollerMotorTorque(double torque) {
    roller.setControl(rollerTorqueCurrentRequest.withOutput(torque));
  }

  @Override
  public void setDeployMotorTorque(double amps) {
    deploy.setControl(deployTorqueCurrentRequest.withOutput(amps));
  }

  public void zeroDeploy() {
    deploy.setPosition(0);
  }
}

package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class IntakeIOTalonFX implements IntakeIO {
  // Motors
  private final TalonFX roller = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);
  private final TalonFX deploy = new TalonFX(IntakeConstants.DEPLOYMENT_MOTOR_ID);

  // Configurators
  private final Slot0Configs deploySlot0 = new Slot0Configs();
  private final MotionMagicConfigs deployMotionMagic = new MotionMagicConfigs();

  // Sensors
  private final CANdi deployLimits = new CANdi(IntakeConstants.DEPLOYMENT_LIMITS_CANDI_ID);
  private final DigitalInput limitIn = new DigitalInput(IntakeConstants.DEPLOYMENT_LIMIT_IN);
  private final DigitalInput limitOut = new DigitalInput(IntakeConstants.DEPLOYMENT_LIMIT_OUT);

  // Control Requests
  private final TorqueCurrentFOC rollerTorqueCurrentRequest =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC deployTorqueCurrentRequest =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VelocityTorqueCurrentFOC deployVelocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final MotionMagicTorqueCurrentFOC deployMotionMagicRequest =
      new MotionMagicTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

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

  private final StatusSignal<Boolean> deployForwardLimit;
  private final StatusSignal<Boolean> deployReverseLimit;

  public IntakeIOTalonFX() {
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_MOTOR_CURRENT_LIMIT;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //    tryUntilOk(5, () -> roller.getConfigurator().apply(rollerConfig, 0.25));

    var deployLimitsConfig = new CANdiConfiguration();
    deployLimitsConfig.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenLow;
    deployLimitsConfig.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenLow;
    deployLimitsConfig.DigitalInputs.S1FloatState = S1FloatStateValue.FloatDetect;
    deployLimitsConfig.DigitalInputs.S2FloatState = S2FloatStateValue.FloatDetect;
    tryUntilOk(5, () -> deployLimits.getConfigurator().apply(deployLimitsConfig));

    var deployConfig = new TalonFXConfiguration();
    deployConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.DEPLOYMENT_MOTOR_CURRENT_LIMIT;
    deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    deployConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //    deployConfig.HardwareLimitSwitch.ForwardLimitSource =
    // ForwardLimitSourceValue.RemoteCANdiS2;
    //    deployConfig.HardwareLimitSwitch.ReverseLimitSource =
    // ReverseLimitSourceValue.RemoteCANdiS1;
    //    deployConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    //    deployConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
    //    deployConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    //    deployConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    //    deployConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    //    deployConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;
    //    deployConfig.HardwareLimitSwitch.ForwardLimitRemoteSensorID =
    //        IntakeConstants.DEPLOYMENT_LIMITS_CANDI_ID;
    //    deployConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID =
    //        IntakeConstants.DEPLOYMENT_LIMITS_CANDI_ID;
    tryUntilOk(5, () -> deploy.getConfigurator().apply(deployConfig, 0.25));

    deploySlot0.kP = IntakeConstants.DEPLOYMENT_KP;
    deploySlot0.kI = IntakeConstants.DEPLOYMENT_KI;
    deploySlot0.kD = IntakeConstants.DEPLOYMENT_KD;
    deploySlot0.kS = IntakeConstants.DEPLOYMENT_KS;
    deploySlot0.kV = IntakeConstants.DEPLOYMENT_KV;
    deploySlot0.kA = IntakeConstants.DEPLOYMENT_KA;

    tryUntilOk(5, () -> deploy.getConfigurator().apply(deploySlot0, 0.25));

    deployMotionMagic.MotionMagicCruiseVelocity = IntakeConstants.DEPLOYMENT_MM_CRUISE_VELOCITY;
    deployMotionMagic.MotionMagicAcceleration = IntakeConstants.DEPLOYMENT_MM_CRUISE_ACCELERATION;
    deployMotionMagic.MotionMagicJerk = IntakeConstants.DEPLOYMENT_MM_CRUISE_JERK;

    tryUntilOk(5, () -> deploy.getConfigurator().apply(deployMotionMagic, 0.25));

    deployForwardLimit = deployLimits.getS2Closed();
    deployReverseLimit = deployLimits.getS1Closed();

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
        deploySupplyCurrent,
        deployForwardLimit,
        deployReverseLimit);

    //    roller.optimizeBusUtilization();
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
        deploySupplyCurrent,
        deployForwardLimit,
        deployReverseLimit);
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
    inputs.deployIn = deployReverseLimit.getValueAsDouble();
    inputs.deployOut = deployForwardLimit.getValueAsDouble();
    ;
  }

  @Override
  public void setRollerMotorTorque(double torque) {
    roller.setControl(rollerTorqueCurrentRequest.withOutput(torque));
  }

  public void applyTunables(IntakeIOTunables tunables) {
    if (Constants.tuningMode) {
      deploySlot0.kP = tunables.deploymentKP;
      deploySlot0.kI = tunables.deploymentKI;
      deploySlot0.kD = tunables.deploymentKD;
      deploySlot0.kS = tunables.deploymentKS;
      deploySlot0.kV = tunables.deploymentKV;
      deploySlot0.kA = tunables.deploymentKA;
      tryUntilOk(5, () -> deploy.getConfigurator().apply(deploySlot0, 0.25));

      deployMotionMagic.MotionMagicCruiseVelocity = tunables.deploymentMMCruiseVelocity;
      deployMotionMagic.MotionMagicAcceleration = tunables.deploymentMMAcceleration;
      deployMotionMagic.MotionMagicJerk = tunables.deploymentMMJerk;
      tryUntilOk(5, () -> deploy.getConfigurator().apply(deployMotionMagic, 0.25));
    }
  }

  @Override
  public void setDeployMotorTorque(double amps) {
    deploy.setControl(
        deployVelocityTorqueCurrentRequest
            .withVelocity(IntakeConstants.DEPLOYMENT_MOTOR_VELOCITY)
            .withFeedForward(amps));
  }

  public void setDeployMotorPosition(double ticks) {
    deploy.setControl(deployMotionMagicRequest.withPosition(ticks));
  }

  public void zeroDeploy() {
    deploy.setPosition(0);
  }
}

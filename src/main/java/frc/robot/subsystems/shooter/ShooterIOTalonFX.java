package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shooterLeader;
  private final TalonFX shooterLeftLower;
  private final TalonFX shooterRightUpper;
  private final TalonFX shooterRightLower;
  private final StatusSignal<Voltage> shooterAppliedVolts;
  private final StatusSignal<AngularVelocity> shooterRPM;
  private final StatusSignal<Current> shooterAmps;

  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
  private final MotionMagicVelocityTorqueCurrentFOC mmVelocityControl =
      new MotionMagicVelocityTorqueCurrentFOC(0).withUpdateFreqHz(0.0);

  public ShooterIOTalonFX() {
    shooterLeader = new TalonFX(ShooterConstants.SHOOTER_LEFT_UPPER);
    shooterLeftLower = new TalonFX(ShooterConstants.SHOOTER_LEFT_LOWER);
    shooterRightUpper = new TalonFX(ShooterConstants.SHOOTER_RIGHT_UPPER);
    shooterRightLower = new TalonFX(ShooterConstants.SHOOTER_RIGHT_LOWER);

    shooterAppliedVolts = shooterLeader.getMotorVoltage();
    shooterRPM = shooterLeader.getVelocity();
    shooterAmps = shooterLeader.getSupplyCurrent();

    var shooterConfig = new TalonFXConfiguration();
    shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_MOTOR_CURRENT_LIMIT;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 1;
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tryUntilOk(5, () -> shooterLeader.getConfigurator().apply(shooterConfig, 0.25));
    tryUntilOk(5, () -> shooterLeftLower.getConfigurator().apply(shooterConfig, 0.25));
    tryUntilOk(5, () -> shooterRightUpper.getConfigurator().apply(shooterConfig, 0.25));
    tryUntilOk(5, () -> shooterRightLower.getConfigurator().apply(shooterConfig, 0.25));

    int leaderID = shooterLeader.getDeviceID();
    // Left lower runs in the same direction as the leader
    shooterLeftLower.setControl(new Follower(leaderID, MotorAlignmentValue.Aligned));
    // Right motors run opposite to the left (inverted)
    shooterRightUpper.setControl(new Follower(leaderID, MotorAlignmentValue.Opposed));
    shooterRightLower.setControl(new Follower(leaderID, MotorAlignmentValue.Opposed));

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = ShooterConstants.kP;
    slot0Configs.kI = ShooterConstants.kI;
    slot0Configs.kD = ShooterConstants.kD;
    slot0Configs.kV = ShooterConstants.kV;
    tryUntilOk(5, () -> shooterLeader.getConfigurator().apply(slot0Configs, 0.25));

    var motionMagicConfigs = shooterConfig.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration =
        400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)
    tryUntilOk(5, () -> shooterLeader.getConfigurator().apply(motionMagicConfigs, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(50, shooterAppliedVolts, shooterRPM, shooterAmps);

    PhoenixUtil.registerSignals(false, shooterAppliedVolts, shooterRPM, shooterAmps);
  }

  public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
    inputs.shooterAppliedVolts = shooterAppliedVolts.getValueAsDouble();
    inputs.shooterRPM = shooterRPM.getValueAsDouble() * 60;
    inputs.shooterSupplyCurrentAmps = shooterAmps.getValueAsDouble();
  }

  public void applyOutputs(ShooterIOOutputs outputs) {
    if (Constants.tuningMode) {
      var slot0Configs = new Slot0Configs();
      slot0Configs.kP = outputs.kP;
      slot0Configs.kI = outputs.kI;
      slot0Configs.kD = outputs.kD;
      slot0Configs.kV = outputs.kV;

      shooterLeader.getConfigurator().apply(slot0Configs);
    }
    if (!outputs.idleDown) {
      setShooterMotorRPM(outputs.velocityRPM);
    } else {
      setShooterMotorVoltage(0);
    }
  }

  @Override
  public void setShooterMotorRPM(double rpm) {
    shooterLeader.setControl(velocityControl.withVelocity(rpm / 60));
  }

  public void setShooterMotorVoltage(double volts) {
    shooterLeader.setVoltage(volts);
  }

  @Override
  public ShooterConstants.ShooterSide getShooterSide() {
    return ShooterConstants.ShooterSide.LEFT; // TODO: Deprecate this
  }
}

package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shooterLeader;
  private final TalonFX shooterFollower;
  private final StatusSignal<Voltage> shooterAppliedVolts;
  private final StatusSignal<AngularVelocity> shooterRPM;
  private final StatusSignal<Current> shooterAmps;

  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);

  public ShooterIOTalonFX(ShooterConstants.ShooterSide side) {
    shooterLeader =
        new TalonFX(
            side.compareTo(ShooterConstants.ShooterSide.LEFT) == 0
                ? ShooterConstants.LeftShooter.SHOOTER_LEADER_MOTOR_ID
                : ShooterConstants.RightShooter.SHOOTER_LEADER_MOTOR_ID);

    shooterFollower =
        new TalonFX(
            side.compareTo(ShooterConstants.ShooterSide.LEFT) == 0
                ? ShooterConstants.LeftShooter.SHOOTER_FOLLOWER_MOTOR_ID
                : ShooterConstants.RightShooter.SHOOTER_FOLLOWER_MOTOR_ID);

    shooterAppliedVolts = shooterLeader.getMotorVoltage();
    shooterRPM = shooterLeader.getVelocity();
    shooterAmps = shooterLeader.getSupplyCurrent();

    var shooterConfig = new TalonFXConfiguration();
    shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_MOTOR_CURRENT_LIMIT;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 10;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    if (side.compareTo(ShooterConstants.ShooterSide.LEFT) == 0)
      shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    else shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // TODO: CHange to coast?
    // shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> shooterLeader.getConfigurator().apply(shooterConfig, 0.25));

    var followerConfig = shooterConfig.clone();
    // followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> shooterFollower.getConfigurator().apply(followerConfig, 0.25));

    shooterFollower.setControl(
        new Follower(shooterLeader.getDeviceID(), MotorAlignmentValue.Aligned));

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = ShooterConstants.kP;
    slot0Configs.kI = ShooterConstants.kI;
    slot0Configs.kD = ShooterConstants.kD;

    shooterLeader.getConfigurator().apply(slot0Configs);

    BaseStatusSignal.setUpdateFrequencyForAll(50, shooterAppliedVolts, shooterRPM, shooterAmps);

    PhoenixUtil.registerSignals(false, shooterAppliedVolts, shooterRPM, shooterAmps);
  }

  public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
    inputs.shooterAppliedVolts = shooterAppliedVolts.getValueAsDouble();
    inputs.shooterRPM = shooterRPM.getValueAsDouble() * 60;
    inputs.shooterAppliedAmps = shooterAmps.getValueAsDouble();
  }

  public void applyOutputs(ShooterIOOutputs outputs) {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = outputs.kP;
    slot0Configs.kI = outputs.kI;
    slot0Configs.kD = outputs.kD;
    slot0Configs.kV = outputs.kV;

    shooterLeader.getConfigurator().apply(slot0Configs);
    shooterLeader.setControl(velocityControl.withVelocity(outputs.velocityRPM / 60));
  }

  /**
   * TODO: Method apply outputs;
   *
   * <p>Take the values from the intakeIOOutputs class and apply them to the leader
   *
   * <p>See
   * https://github.com/Mechanical-Advantage/RobotCode2024Public/blob/main/src/main/java/org/littletonrobotics/frc2024/subsystems/flywheels/FlywheelsIOKrakenFOC.java#L157
   * This doesn't use the latest phoenix API, you'll need to update it
   */
  @Override
  public void setShooterMotorRPS(double rpm) {
    shooterLeader.setControl(velocityControl.withVelocity(rpm));
  }
}

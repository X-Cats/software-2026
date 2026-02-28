package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shooterLeader;
  private final TalonFX shooterFollower;
  private final StatusSignal<Voltage> shooterAppliedVolts;

  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV);

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

    var shooterConfig = new TalonFXConfiguration();
    shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_MOTOR_CURRENT_LIMIT;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    if (side.compareTo(ShooterConstants.ShooterSide.LEFT) == 0)
      shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    else shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // TODO: CHange to coast?
    // shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterLeader.getConfigurator().apply(shooterConfig, 0.25);
    tryUntilOk(5, () -> shooterFollower.getConfigurator().apply(shooterConfig, 0.25));

    var followerConfig = shooterConfig.clone();
    // followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterFollower.getConfigurator().apply(followerConfig, 0.25);
    shooterFollower.setControl(
        new Follower(shooterLeader.getDeviceID(), MotorAlignmentValue.Aligned));

    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = ShooterConstants.kS;
    slot0Configs.kV = ShooterConstants.kV;
    slot0Configs.kP = ShooterConstants.kP;
    slot0Configs.kI = ShooterConstants.kI;
    slot0Configs.kD = ShooterConstants.kD;

    shooterLeader.getConfigurator().apply(slot0Configs);
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
  public void setShooterMotorRPM(double rpm) {
    shooterLeader.setControl(velocityControl.withVelocity(rpm));
  }
}

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shooterLeader;
  private final TalonFX shooterFollower;
  private final StatusSignal<Voltage> shooterAppliedVolts;

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
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterLeader.getConfigurator().apply(shooterConfig, 0.25);
    // tryUntilOk(5, ()-> shooterFollower.getConfigurator().apply(shooterConfig, 0.25));
    var followerConfig = shooterConfig.clone();
    followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterFollower.getConfigurator().apply(shooterConfig, 0.25);
    shooterLeader.setControl(
        new Follower(shooterFollower.getDeviceID(), MotorAlignmentValue.Aligned));
  }
}

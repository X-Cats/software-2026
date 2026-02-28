package frc.robot.subsystems.shooter;

public class ShooterConstants {
  public enum ShooterSide {
    LEFT,
    RIGHT
  }

  public static final double SHOOTER_MOTOR_VOLTAGE = 1;
  public static final double SHOOTER_MOTOR_REDUCTION = 1;
  public static final double SHOOTER_MOTOR_CURRENT_LIMIT = 1;

  public static class LeftShooter {
    public static final int SHOOTER_LEADER_MOTOR_ID = 50;
    public static final int SHOOTER_FOLLOWER_MOTOR_ID = 51;
  }

  public static class RightShooter {
    public static final int SHOOTER_LEADER_MOTOR_ID = 52;
    public static final int SHOOTER_FOLLOWER_MOTOR_ID = 53;
  }
}

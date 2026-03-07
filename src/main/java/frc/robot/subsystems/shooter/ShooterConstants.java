package frc.robot.subsystems.shooter;

public class ShooterConstants {
  public enum ShooterSide {
    LEFT,
    RIGHT
  }

  public static final double SHOOTER_MOTOR_VOLTAGE = 1;
  public static final double SHOOTER_MOTOR_REDUCTION = 1;
  public static final double SHOOTER_MOTOR_CURRENT_LIMIT = 40;
  public static final double kP = 0.65;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV = 0.125;

  public static class LeftShooter {
    public static final int SHOOTER_LEADER_MOTOR_ID = 50;
    public static final int SHOOTER_FOLLOWER_MOTOR_ID = 51;
  }

  public static class RightShooter {
    public static final int SHOOTER_LEADER_MOTOR_ID = 52;
    public static final int SHOOTER_FOLLOWER_MOTOR_ID = 53;
  }
}

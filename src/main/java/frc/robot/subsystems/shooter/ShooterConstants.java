package frc.robot.subsystems.shooter;

public class ShooterConstants {

  public static final double SHOOTER_MOTOR_VOLTAGE = 1;
  public static final double SHOOTER_MOTOR_REDUCTION = 1;
  public static final double SHOOTER_MOTOR_CURRENT_LIMIT = 10;
  public static final double SHOOTER_RAMP_RATE = 20;
  public static final double kP = 0.4;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV = 0.125;

  public static final int SHOOTER_LEFT_UPPER = 50;
  public static final int SHOOTER_LEFT_LOWER = 51;
  public static final int SHOOTER_RIGHT_UPPER = 52;
  public static final int SHOOTER_RIGHT_LOWER = 53;

  public enum ShooterSide {
    LEFT,
    RIGHT,
    SIM
  }
}

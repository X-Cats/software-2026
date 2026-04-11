package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static final int DEPLOYMENT_MOTOR_ID = 5;
  public static final int DEPLOYMENT_MOTOR_CURRENT = 15;
  public static final int DEPLOYMENT_MOTOR_STOW_CURRENT = 15;
  public static final int DEPLOYMENT_MOTOR_VELOCITY = 20;
  public static final double DEPLOYMENT_MOTOR_REDUCTION = 8.951;
  public static final double DEPLOYMENT_MOTOR_CURRENT_LIMIT = 20;
  public static final int DEPLOYMENT_LIMITS_CANDI_ID = 6;
  public static final int DEPLOYMENT_LIMIT_IN = 1;
  public static final int DEPLOYMENT_LIMIT_OUT = 0;

  public static final double DEPLOYMENT_KS = 4.0;
  public static final double DEPLOYMENT_KV = 0.2;
  public static final double DEPLOYMENT_KA = 0.0;
  public static final double DEPLOYMENT_KP = 20;
  public static final double DEPLOYMENT_KI = 0.0;
  public static final double DEPLOYMENT_KD = 5.5;

  public static final double DEPLOYMENT_MM_CRUISE_VELOCITY = 40;
  public static final double DEPLOYMENT_MM_CRUISE_ACCELERATION = 80;
  public static final double DEPLOYMENT_MM_CRUISE_JERK = 1000;

  public static final double DEPLOYMENT_DEPLOYED_SETPOINT = 30.0; // TODO: Fill in tomorrow
  public static final double DEPLOYMENT_STOWED_SETPOINT = 5.0;

  public static final int ROLLER_MOTOR_ID = 4;
  public static final int ROLLER_MOTOR_TORQUE = 40;
  public static final double ROLLER_MOTOR_REDUCTION = 1;
  public static final double ROLLER_MOTOR_CURRENT_LIMIT = 40;
}

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.ShooterSide.SIM;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double shooterAppliedVolts = 0.0;
    public double shooterSupplyCurrentAmps = 0.0;
    public double shooterRPM = 0.0;
  }

  public static enum ShooterIOOutputMode {
    COAST,
    VELOCITY
  }

  public static class ShooterIOOutputs {
    public ShooterIOOutputMode mode = ShooterIOOutputMode.COAST;

    // TODO: Add values for kP, kI, kD, FF, and Velocity here
    public double velocityRPM;
    public double feedforward = 0.0;
    public double kP;
    public double kI;
    public double kD;
    public double kV;
    public boolean idleDown = false;
  }

  // Dont touch these ******
  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void applyOutputs(ShooterIOOutputs outputs) {}

  // Put your hands on these *****
  public default void setShooterMotorRPM(double RPM) {}
  public default void setShooterMotorVoltage(double volts) {}

  public default ShooterConstants.ShooterSide getShooterSide() {
    return SIM;
  }
}

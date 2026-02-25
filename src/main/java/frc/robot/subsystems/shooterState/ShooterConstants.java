package frc.robot.subsystems.shooterState;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
	public static final InterpolatingDoubleTreeMap BALL_TO_FLYWHEEL_SPEED = new InterpolatingDoubleTreeMap();
	public static final double GRAVITY = 9.81; // m s^-2
	public static final double FINAL_VERTICAL_VELOCITY = -1; // m s^-1
	public static final double HUB_HEIGHT; // TODO: vertical offset to hub in meters
	public static final double FLIGHT_TIME =
		(Math.sqrt(FINAL_VERTICAL_VELOCITY*FINAL_VERTICAL_VELOCITY + 2*GRAVITY*HUB_HEIGHT) - FINAL_VERTICAL_VELOCITY) / GRAVITY;
}

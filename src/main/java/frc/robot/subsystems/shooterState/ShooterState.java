package frc.robot.subsystems.shooterState;

import frc.robot.Localization;

public class ShooterState {
	private double pitch;
	private double heading;
	private double speed;
	private Localization localization;

	public ShooterState(Localization localization) {
		this.localization = localization;
	}

	public double pitch() {
		return pitch;
	}

	public double heading() {
		return heading;
	}

	/**
	 * @return the speed at which the flywheels need to spin
	 */
	public double speed() {
		return ShooterConstants.BALL_TO_FLYWHEEL_SPEED.get(speed);
	}

	public void periodic() {
		double robotVX; // robot velocity in meters per second
		double robotVY; // robot velocity in meters per second
		double effectiveDX = hubX-robotX-ShooterConstants.FLIGHT_TIME*robotVX;
		double effectiveDY = hubY-robotY-ShooterConstants.FLIGHT_TIME*robotVY;
		double shotVx = effectiveDX/ShooterConstants.FLIGHT_TIME;
		double shotVy = effectiveDY/ShooterConstants.FLIGHT_TIME;
		double shotVz = ShooterConstants.HUB_HEIGHT/ShooterConstants.FLIGHT_TIME + ShooterConstants.FLIGHT_TIME*ShooterConstants.GRAVITY/2;
		heading = Math.acos(effectiveDX / Math.sqrt(effectiveDX*effectiveDX+effectiveDY*effectiveDY));
		speed = Math.sqrt(shotVx*shotVx + shotVy*shotVy + shotVz*shotVz);
		pitch = Math.asin(shotVz / speed);
	}
}

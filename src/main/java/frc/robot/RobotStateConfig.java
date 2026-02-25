package frc.robot;

public class RobotStateConfig {

  public enum SuperState {
    IDLE("IDLE"),
    INTAKING("INTAKING"),
    SHOOTING("SHOOTING"),
    AGITATING("AGITATING"),
    ZERO("ZERO"); // EMPTY STATE Means do Nothing.....

    private final String prettyName;

    SuperState(String idle) {
      prettyName = idle;
    }

    @Override
    public String toString() {
      return "Enum SuperState: " + prettyName;
    }
  }
}

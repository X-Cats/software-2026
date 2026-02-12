package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runEnd;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLog;

// author Daniel Rabess
public class RobotState {

  private final DesiredIntakeState dis = new DesiredIntakeState();

  public RobotState() {}

  public RobotState(String probablyLater) {}

  public Command runIntake() {
    return runEnd(
        () -> {
          dis.setExtension(DesiredIntakeState.IntakeExtensionState.EXTENDED);
          dis.setRunRoller(true);
        },
        () -> {
          dis.setExtension(DesiredIntakeState.IntakeExtensionState.RETRACTED);
          dis.setRunRoller(false);
        });
  }

  public DesiredIntakeState getDesiredIntakeState() {
    return dis;
  }

  @AutoLog
  public static class DesiredIntakeState {

    public enum IntakeExtensionState {
      EXTENDED,
      RETRACTED
    }

    public DesiredIntakeState() {}

    public IntakeExtensionState extension = IntakeExtensionState.RETRACTED;
    public boolean runRoller = false;

    public IntakeExtensionState getExtension() {
      return extension;
    }

    private void setExtension(IntakeExtensionState extension) {
      this.extension = extension;
    }

    public boolean getRunRoller() {
      return runRoller;
    }

    private void setRunRoller(boolean runRoller) {
      this.runRoller = runRoller;
    }
  }
}

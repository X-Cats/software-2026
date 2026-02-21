package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runEnd;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLog;

// author Daniel Rabess
public class RobotStateMachine {

  private final DesiredIntakeState dIntakeState = new DesiredIntakeState();
  private final DesiredShooterState dShooterState = new DesiredShooterState();
  private final DesiredConveyorState dConveyorState = new DesiredConveyorState();
  private final DesiredHoodState dHoodState = new DesiredHoodState();

  public RobotStateMachine() {}

  public RobotStateMachine(String probablyLater) {}

  public Command runIntake() {
    return runEnd(
        () -> {
          dIntakeState.setExtension(DesiredIntakeState.IntakeExtensionState.EXTENDED);
          dIntakeState.setRunRoller(true);
        },
        () -> {
          dIntakeState.setExtension(DesiredIntakeState.IntakeExtensionState.RETRACTED);
          dIntakeState.setRunRoller(false);
        });
  }

  public Command runShooter() {
    return runEnd(
        () -> {
          dShooterState.setShooterMode(DesiredShooterState.ShooterModeState.SUPPRESSED);
        },
        () -> {
          dShooterState.setShooterMode(DesiredShooterState.ShooterModeState.ON);
        });
  }

  public Command runHopper() {
    return runEnd(
        () -> {
          dConveyorState.setConveyorState(DesiredConveyorState.ConveyorState.FEEDING);
        },
        () -> {
          dConveyorState.setConveyorState(DesiredConveyorState.ConveyorState.OFF);
        });
  }

  public Command runHood() {
    return runEnd(
        () -> {
          dHoodState.setHoodState(DesiredHoodState.HoodState.AIMING);
        },
        () -> {
          dHoodState.setHoodState(DesiredHoodState.HoodState.STOWED);
        });
  }

  public DesiredIntakeState getDesiredIntakeState() {
    return dIntakeState;
  }

  public DesiredShooterState getDesiredShooterState() {
    return dShooterState;
  }

  public DesiredConveyorState getDesiredConveyorState() {
    return dConveyorState;
  }

  public DesiredHoodState getDesiredHoodState() {
    return dHoodState;
  }

  // Desired states
  // ============================================================================================

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

  @AutoLog
  public static class DesiredShooterState {

    public enum ShooterModeState {
      ON, // Shooter Motor is On - Kicker Motor is On
      SUPPRESSED, // Shooter Motor is On - Kicker Motor is Off
      OFF // Shooter Motor is Off - Kicker Motor is Off
    }

    public DesiredShooterState() {}

    public ShooterModeState shooterMode = ShooterModeState.ON;

    public ShooterModeState getShooterMode() {
      return shooterMode;
    }

    public void setShooterMode(ShooterModeState shooterMode) {
      this.shooterMode = shooterMode;
    }
  }

  @AutoLog
  public static class DesiredConveyorState {

    public ConveyorState getConveyorState() {
      return conveyorState;
    }

    public void setConveyorState(ConveyorState conveyorState) {
      this.conveyorState = conveyorState;
    }

    public enum ConveyorState {
      FEEDING,
      SHUFFLING,
      EJECTING,
      OFF
    }

    public DesiredConveyorState() {}

    public ConveyorState conveyorState = ConveyorState.OFF;
  }

  @AutoLog
  public static class DesiredHoodState {

    public HoodState getHoodState() {
      return hoodState;
    }

    public void setHoodState(HoodState hoodState) {
      this.hoodState = hoodState;
    }

    public enum HoodState {
      AIMING,
      STOWED
    }

    public DesiredHoodState() {}

    public HoodState hoodState = HoodState.STOWED;
  }
}

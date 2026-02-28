package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

// author Daniel Rabess
public class RobotStateMachine extends SubsystemBase {

  private final DesiredIntakeStateAutoLogged dIntakeState = new DesiredIntakeStateAutoLogged();
  private final DesiredShooterStateAutoLogged dShooterState = new DesiredShooterStateAutoLogged();
  private final DesiredConveyorStateAutoLogged dConveyorState =
      new DesiredConveyorStateAutoLogged();
  private final DesiredHoodStateAutoLogged dHoodState = new DesiredHoodStateAutoLogged();

  // State we want to transition too
  private RobotStateConfig.SuperState desiredSuperState;
  // State the robot is currently configured for
  private RobotStateConfig.SuperState currentSuperState;

  public RobotStateMachine() {
    currentSuperState = RobotStateConfig.SuperState.IDLE;
    desiredSuperState = RobotStateConfig.SuperState.ZERO; // Do nothing to for now...
  }

  public RobotStateMachine(String probablyLater) {}

  @Override
  public void periodic() {
    // io.updateInputs(inputs);
    // Logger.processInputs("Robot State Machine", inputs);
    Logger.processInputs("RobotState", dIntakeState);
    Logger.processInputs("RobotState", dShooterState);
    Logger.processInputs("RobotState", dConveyorState);
    Logger.processInputs("RobotState", dHoodState);
    updateSuperState();
  }

  public void setDesiredSuperState(RobotStateConfig.SuperState dss) {
    this.desiredSuperState = dss;
  }

  public void setCurrentSuperState(RobotStateConfig.SuperState css) {
    this.currentSuperState = css;
  }

  public RobotStateConfig.SuperState getDesiredSuperState() {
    return this.desiredSuperState;
  }

  public RobotStateConfig.SuperState getCurrentSuperState() {
    return this.currentSuperState;
  }

  // Returns False if State Cannot be updated
  public boolean updateSuperState() {
    boolean retval = false;

    switch (getDesiredSuperState()) {
      case IDLE:
        retval = transitionIDLE();
        break;
      case INTAKING:
        retval = transitionINTAKING();
        break;
      case SHOOTING:
        retval = transitionSHOOTING();
        break;
      case AGITATING:
        retval = transitionAGITATING();
        break;

      default:
        retval = transitionIDLE();
        break;
    }

    return retval;
  }

  // State Transition Helper Functions
  public boolean transitionIDLE() {
    return true;
  }

  public boolean transitionINTAKING() {

    // Call safety functions
    ensureHoodIsStowed();

    dShooterState.setShooterMode(DesiredShooterState.ShooterModeState.OFF);
    dIntakeState.setDesiredIntakeRollerState(DesiredIntakeState.IntakeRollerState.INTAKING);
    dIntakeState.setDesiredIntakeDeployState(DesiredIntakeState.IntakeDeployState.DEPLOYED);
    dConveyorState.setConveyorState(DesiredConveyorState.ConveyorState.CONVEYING);
    // dHoodState.setHoodState(DesiredHoodState.HoodState.STOWED);
    dHoodState.setKickerState(DesiredHoodState.KickerState.INDEXING);
    return true;
  }

  public boolean transitionSHOOTING() {
    dShooterState.setShooterMode(DesiredShooterState.ShooterModeState.ON);
    dIntakeState.setDesiredIntakeRollerState(DesiredIntakeState.IntakeRollerState.INTAKING);
    dIntakeState.setDesiredIntakeDeployState(DesiredIntakeState.IntakeDeployState.DEPLOYED);
    dConveyorState.setConveyorState(DesiredConveyorState.ConveyorState.CONVEYING);
    dHoodState.setHoodState(DesiredHoodState.HoodState.AIMING);
    dHoodState.setKickerState(DesiredHoodState.KickerState.FEEDING);

    return true;
  }

  public boolean transitionAGITATING() {

    ensureHoodIsStowed();

    dShooterState.setShooterMode(DesiredShooterState.ShooterModeState.OFF);
    dIntakeState.setDesiredIntakeRollerState(DesiredIntakeState.IntakeRollerState.INTAKING);
    dIntakeState.setDesiredIntakeDeployState(DesiredIntakeState.IntakeDeployState.DEPLOYED);
    dConveyorState.setConveyorState(DesiredConveyorState.ConveyorState.CONVEYING);
    // dHoodState.setHoodState(DesiredHoodState.HoodState.STOWED);
    dHoodState.setKickerState(DesiredHoodState.KickerState.INDEXING);

    return true;
  }

  public boolean transitionZERO() {

    return true;
  }

  // ** SAFETY HELPER FUNCTIONS - Make Functions here for state safety **//

  // This function will check that the hood is stowed
  // and if its NOT stowed, will change state to be STOWED.
  // returns true if Hood State was changed to STOWED
  public boolean ensureHoodIsStowed() {
    boolean retval = false;

    if (dHoodState.getHoodState() != DesiredHoodState.HoodState.STOWED) {
      dHoodState.setHoodState(DesiredHoodState.HoodState.STOWED);
      retval = true;
    } else {
      retval = true;
    }

    return retval;
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

    public enum IntakeRollerState {
      INTAKING,
      EJECTING,
      OFF
    }

    public IntakeRollerState intakeRollerState = IntakeRollerState.INTAKING;

    public enum IntakeDeployState {
      DEPLOYED,
      STOWED,
      OFF
    }

    public IntakeDeployState intakeDeployState = IntakeDeployState.DEPLOYED;

    public void setDesiredIntakeRollerState(IntakeRollerState irs) {
      this.intakeRollerState = irs;
    }

    public IntakeRollerState getDesiredIntakeRollerState() {
      return this.intakeRollerState;
    }

    public void setDesiredIntakeDeployState(IntakeDeployState ids) {
      this.intakeDeployState = ids;
    }

    public IntakeDeployState getDesiredIntakeDeployState() {
      return this.intakeDeployState;
    }

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
      CONVEYING,
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
      STOWED,
      ZERO
    }

    public enum KickerState {
      FEEDING,
      INDEXING,
      OFF
    }

    public KickerState kickerState = KickerState.FEEDING;

    public KickerState getKickerState() {
      return kickerState;
    }

    public void setKickerState(KickerState ks) {
      this.kickerState = ks;
    }

    public DesiredHoodState() {}

    public HoodState hoodState = HoodState.STOWED;
  }
}

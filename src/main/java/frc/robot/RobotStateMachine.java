package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

// author Daniel Rabess
public class RobotStateMachine extends SubsystemBase {

  private final DesiredIntakeStateAutoLogged dIntakeState = new DesiredIntakeStateAutoLogged();
  private final DesiredShooterStateAutoLogged dShooterState = new DesiredShooterStateAutoLogged();
  private final ShooterStateAutoLogged aShooterState = new ShooterStateAutoLogged();
  private final DesiredConveyorStateAutoLogged dConveyorState =
      new DesiredConveyorStateAutoLogged();
  private final DesiredHoodStateAutoLogged dHoodKickerState = new DesiredHoodStateAutoLogged();

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
    Logger.processInputs("RobotState", dHoodKickerState);
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
        retval = transitionIdle();
        break;
      case INTAKING:
        retval = transitionIntaking();
        break;
      case SHOOTING:
        retval = transitionShooting();
        break;
      case AGITATING:
        retval = transitionAgitating();
        break;

      default:
        retval = transitionIdle();
        break;
    }

    return retval;
  }

  // State Transition Helper Functions
  public boolean transitionIdle() {
    dIntakeState.setDesiredIntakeRollerState(DesiredIntakeState.IntakeRollerState.OFF);
    dIntakeState.setDesiredIntakeDeployState(DesiredIntakeState.IntakeDeployState.STOWED);

    dConveyorState.setConveyorState(DesiredConveyorState.ConveyorState.OFF);

    dHoodKickerState.setKickerState(DesiredHoodState.KickerState.OFF);
    dHoodKickerState.setHoodState(DesiredHoodState.HoodState.STOWED);

    dShooterState.setShooterMode(DesiredShooterState.ShooterModeState.IDLE);
    return true;
  }

  public boolean transitionIntaking() {
    dIntakeState.setDesiredIntakeRollerState(DesiredIntakeState.IntakeRollerState.INTAKING);
    dIntakeState.setDesiredIntakeDeployState(DesiredIntakeState.IntakeDeployState.DEPLOYED);

    dConveyorState.setConveyorState(DesiredConveyorState.ConveyorState.OFF);

    dHoodKickerState.setKickerState(DesiredHoodState.KickerState.OFF);
    dHoodKickerState.setHoodState(DesiredHoodState.HoodState.STOWED);

    dShooterState.setShooterMode(DesiredShooterState.ShooterModeState.IDLE);

    return true;
  }

  public boolean transitionShooting() {
    dIntakeState.setDesiredIntakeRollerState(DesiredIntakeState.IntakeRollerState.INTAKING);
    dIntakeState.setDesiredIntakeDeployState(DesiredIntakeState.IntakeDeployState.STOWED);

    dShooterState.setShooterMode(DesiredShooterState.ShooterModeState.ON);

    dHoodKickerState.setHoodState(DesiredHoodState.HoodState.AIMING);
    dHoodKickerState.setKickerState(DesiredHoodState.KickerState.FEEDING);

    dConveyorState.setConveyorState(DesiredConveyorState.ConveyorState.CONVEYING);

    return true;
  }

  public boolean transitionAgitating() {
    dHoodKickerState.setKickerState(DesiredHoodState.KickerState.INDEXING);
    dIntakeState.setDesiredIntakeDeployState(DesiredIntakeState.IntakeDeployState.DEPLOYED);

    dConveyorState.setConveyorState(DesiredConveyorState.ConveyorState.EJECTING);

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

    if (dHoodKickerState.getHoodState() != DesiredHoodState.HoodState.STOWED) {
      dHoodKickerState.setHoodState(DesiredHoodState.HoodState.STOWED);
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
    return dHoodKickerState;
  }

  public ShooterState getShooterState() {
    return this.aShooterState;
  }

  public boolean getShooterAssyReady() {
    // TODO: Add hood here too
    return this.aShooterState.shooterAtSpeed;
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
      IDLE, // Shooter Motor is On - Kicker Motor is Off
      OFF // Shooter Motor is Off - Kicker Motor is Off
    }

    public DesiredShooterState() {}

    public ShooterModeState shooterMode = ShooterModeState.OFF;

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

  @AutoLog
  public static class ShooterState {
    public boolean shooterAtSpeed;

    public void setShooterAtSpeed(boolean atSpeed) {
      this.shooterAtSpeed = atSpeed;
    }
  }
}

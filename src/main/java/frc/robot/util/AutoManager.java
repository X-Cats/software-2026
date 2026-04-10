package frc.robot.util;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoManager {
  private SendableChooser<Command> autos;

  public AutoManager() {
    autos = new SendableChooser<>();
    autos.addOption("LHS Trench Run", new PathPlannerAuto("LHS Theirs and Ours"));
    autos.addOption("LHS Wait Trench", new PathPlannerAuto("LHS Wait and Theirs"));
    autos.addOption("RHS Trench Run", new PathPlannerAuto("LHS Theirs and Ours", true));
    autos.addOption("RHS Wait Trench", new PathPlannerAuto("LHS Wait and Theirs", true));
  }

  public SendableChooser<Command> getChooser() {
    return autos;
  }
}

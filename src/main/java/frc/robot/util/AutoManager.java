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
    autos.addOption("LHS CHEEEEEESE", new PathPlannerAuto("Cheesy Copy"));
    autos.addOption("LHS Cheese Pass", new PathPlannerAuto("Cheesy Pass"));
    autos.addOption("RHS Trench Run", new PathPlannerAuto("LHS Theirs and Ours", true));
    autos.addOption("RHS Wait Trench", new PathPlannerAuto("LHS Wait and Theirs", true));
    autos.addOption("RHS CHEEEEEESE", new PathPlannerAuto("Cheesy Copy", true));
    autos.addOption("RHS Cheese Pass", new PathPlannerAuto("Cheesy Pass", true));
  }

  public SendableChooser<Command> getChooser() {
    return autos;
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotContainer;
import frc.robot.utilities.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command. You can replace me with your own command.
 */
public class DisplayLogCommand extends CommandBase {
  private String s;

  public static enum DISPLAYMODE {
    NORM, INIT_TIME, ELASPED_TIME
  }

  DISPLAYMODE mode = DISPLAYMODE.NORM;

  public DisplayLogCommand(String s, DISPLAYMODE mode) {
    this.s = s;
    this.mode = mode;
  }

  public DisplayLogCommand(String s) {
    // Use requires() here to declare subsystem dependencies
    this.s = s;
    mode = DISPLAYMODE.NORM;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    switch (mode) {
      case NORM:
        Util.logf("%s\n", s);
        break;
      case INIT_TIME:
        RobotContainer.autonomousInitTime = RobotController.getFPGATime();
        Util.logf("********* Start %s Time:%.3f\n", s, RobotContainer.autonomousInitTime / 1000000.0);
        break;
      case ELASPED_TIME:
        long completeTime = RobotController.getFPGATime();
        Util.logf("********** End %s Current Time:%.3f Elapsed:%.3f\n", s, completeTime / 1000000.0,
            (completeTime - RobotContainer.autonomousInitTime) / 1000000.0);
        break;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

}
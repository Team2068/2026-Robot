package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.DistanceShootUtil;
import frc.robot.utility.IO;

public class DistanceShoot extends Command {
  IO io;

  DistanceShootUtil[] data = {new DistanceShootUtil(0, 0, 0)};

  public DistanceShoot(IO io) {
    addRequirements(io.flywheel);
  }

  @Override
  public void initialize() {
    // Set flywheel RPM and hood angle based on distance
  }

  @Override
  public void execute() {
    // Check if flywheel has hit RPM
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

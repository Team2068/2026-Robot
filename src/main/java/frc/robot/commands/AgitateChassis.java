package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;

public class AgitateChassis extends Command {
  IO io;
  Timer timer = new Timer();
  boolean end;

  public AgitateChassis(IO io) {
    this.io = io;
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    io.chassis.drive(new ChassisSpeeds(0.0, 0.0, 3));
    if(timer.get() >= 0.6)
      end = true;
  }

  @Override
  public void end(boolean interrupted) {
    io.chassis.stop();
    timer.restart();
    end = false;
  }

  @Override
  public boolean isFinished() {
    return end;
  }
}

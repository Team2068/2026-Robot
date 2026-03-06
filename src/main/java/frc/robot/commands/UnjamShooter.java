package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;

public class UnjamShooter extends Command {
  IO io;

  public UnjamShooter(IO io) {
    this.io = io;
    addRequirements(io.flywheel, io.feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    io.feeder.speed(0.75);
    io.flywheel.flywheelSpeed(-0.75);
    io.feeder.unblock();
  }

  @Override
  public void end(boolean interrupted) {
    io.feeder.stop();
    io.feeder.block();
    io.flywheel.stopFlywheel();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

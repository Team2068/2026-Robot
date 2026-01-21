package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;

public class RotateChassis extends Command {
  IO io;
  PIDController pid = new PIDController(0, 0, 0);
  double target;

  public RotateChassis(IO io, double target) {
    this.io = io;
    addRequirements(io.chassis);
    this.target = target;
  }

  @Override
  public void initialize() {
    pid.setTolerance(0.05);
    pid.enableContinuousInput(0, 360);
  }

  @Override
  public void execute() {
    io.chassis.drive(new ChassisSpeeds(0, 0, pid.calculate(io.chassis.getYaw(), target)));
  }

  @Override
  public void end(boolean interrupted) {
    io.chassis.drive(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}

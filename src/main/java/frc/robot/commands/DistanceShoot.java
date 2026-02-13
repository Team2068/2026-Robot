package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.utility.DistanceShootUtil;
import frc.robot.utility.IO;

public class DistanceShoot extends Command {
  IO io;
  boolean blue;
  int rpmTolerance = 50;
  double angleTolerance = 1.5;

  DistanceShootUtil[] data = { new DistanceShootUtil(0, 0, 0) };

  public DistanceShoot(IO io) {
    addRequirements(io.flywheel);
  }

  @Override
  public void initialize() {
    blue = DriverStation.getAlliance().get() == Alliance.Blue;
  }

  @Override
  public void execute() {
    Translation2d hub = blue ? RobotContainer.BLUE_HUB : RobotContainer.RED_HUB;
    Translation2d diff = hub.minus(io.chassis.pose().getTranslation());
    DistanceShootUtil helper = calculateFlywheel(Math.hypot(diff.getY(), diff.getX()));

    io.flywheel.hoodAngle(helper.hoodAngle);
    io.flywheel.RPM(helper.shooterRPM);

    if (Math.abs(io.flywheel.RPM() - helper.shooterRPM) < rpmTolerance && Math.abs(io.flywheel.hoodAngle() - helper.hoodAngle) < angleTolerance) {
      io.feeder.speed(0.5);
    }
  }

  public DistanceShootUtil calculateFlywheel(double distance) {
    DistanceShootUtil lower = data[0];
    DistanceShootUtil upper = data[data.length - 1];

    for (int i = 0; i < data.length - 1; i++) {
      if (distance >= data[i].distance && distance <= data[i + 1].distance) {
        lower = data[i];
        upper = data[i + 1];
        break;
      }
    }

    double factor = (distance - lower.distance) / (upper.distance - lower.distance);
    factor = MathUtil.clamp(factor, 0, 1);

    double hood = lower.hoodAngle + factor * (upper.hoodAngle - lower.hoodAngle);
    double rpm = lower.shooterRPM + factor * (upper.shooterRPM - lower.shooterRPM);

    return new DistanceShootUtil(distance, hood, rpm);
  }

  @Override
  public void end(boolean interrupted) {
    io.flywheel.stopFlywheel();
    io.flywheel.stopHood();
    io.feeder.stop();
  }

  @Override
  public boolean isFinished() {
    return io.feeder.supplied() == false;
  }
}

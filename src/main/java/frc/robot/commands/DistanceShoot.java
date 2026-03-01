package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.swerveState;
import frc.robot.utility.DistanceShootUtil;
import frc.robot.utility.IO;

public class DistanceShoot extends Command {
  IO io;
  boolean blue;
  // TODO tune shooter pid so tolerance can be lower
  private static final int RPM_TOLERANCE = 120;
  private static final double ANGLE_TOLERANCE = 1.5;

  Timer timer = new Timer();

  DistanceShootUtil[] data = {new DistanceShootUtil(0, 2, 4400), new DistanceShootUtil(1.67, 4, 4500), new DistanceShootUtil(2.05, 6, 4600), new DistanceShootUtil(2.33, 8, 4600), new DistanceShootUtil(2.75, 16, 4900), new DistanceShootUtil(3.5, 19, 5200), new DistanceShootUtil(4.0, 21, 5400), new DistanceShootUtil(4.9, 28, 5900) };

  DistanceShootUtil distanceUtil = null;

  public DistanceShoot(IO io) {
    this.io = io;
    addRequirements(io.flywheel, io.feeder);
  }

  public DistanceShoot(IO io, DistanceShootUtil distanceUtil) {
    this.io = io;
    addRequirements(io.flywheel, io.feeder);
    this.distanceUtil = distanceUtil;
  }

  @Override
  public void initialize() {
    blue = DriverStation.getAlliance().get() == Alliance.Blue;
    timer.start();
    timer.restart();
  }

  @Override
  public void execute() {
    DistanceShootUtil helper;

    if (io.chassis.currentState != swerveState.PASSING) {
      if (distanceUtil == null) {
        Translation2d hub = blue ? RobotContainer.BLUE_HUB : RobotContainer.RED_HUB;
        Translation2d diff = hub.minus(io.chassis.getEstimatedPose().getTranslation());
        helper = calculateFlywheel(Math.hypot(diff.getY(), diff.getX()));
      } else {
        helper = distanceUtil;
      }
    } else {
      // TODO Find optimal values for passing from mid field
      helper = new DistanceShootUtil(0, 6000);
    }

    io.flywheel.hoodAngle(helper.hoodAngle);
    io.flywheel.RPM(helper.shooterRPM);

    if (Math.abs(io.flywheel.RPM() - helper.shooterRPM) < RPM_TOLERANCE
        && Math.abs(io.flywheel.hoodAngle() - helper.hoodAngle) < ANGLE_TOLERANCE) {
      io.feeder.voltLoop(7.2);
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
    return false;
  }
}

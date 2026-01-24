package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.swerveState;
import frc.robot.utility.IO;

public class Aimbot extends Command {
  IO io;
  swerveState state;
  PIDController pid = new PIDController(0.01, 0, 0.001);
  boolean blue;
  double target;

  // Hub cords, will likely move to a constants folder since these values will be
  // needed for the distance shoot
  private static final Translation2d BLUE_HUB = new Translation2d(-3.6475, 0); // TODO get positions
  private static final Translation2d RED_HUB = new Translation2d(3.6423, 0);
//Estimated directly from Limelight Andymark Map (in meters)

  public Aimbot(IO io, swerveState state) {
    this.io = io;
    blue = DriverStation.getAlliance().get() == Alliance.Blue;
  }

  @Override
  public void initialize() {
    io.chassis.currentState = state;
    pid.setTolerance(Math.toRadians(1));
    pid.enableContinuousInput(-Math.PI, Math.PI);
    pid.reset();
  }
  
  @Override
  public void execute() {
    // TODO Get all values
    if (state == swerveState.PASSING) {
      target = blue ? Math.toRadians(0) : Math.toRadians(180);
    } else if (state == swerveState.SCORING) {
      Translation2d hub = blue ? BLUE_HUB : RED_HUB;
      Translation2d diff = hub.minus(io.chassis.pose().getTranslation());
      target = Math.atan2(diff.getY(), diff.getX());
    }

    if(pid.atSetpoint()){
      io.chassis.targetRotation = 0.0;
    }
    else{
      io.chassis.targetRotation = pid.calculate(Math.toRadians(io.chassis.getYaw()), target); // TODO might need to use MathUtil.clamp to make it less jittery
    }
  }

  @Override
  public void end(boolean interrupted) {
    io.chassis.targetRotation = null;
  }

  @Override
  public boolean isFinished() {
    return (io.chassis.currentState != state);
  }
}

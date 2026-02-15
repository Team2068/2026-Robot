package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.swerveState;
import frc.robot.utility.IO;

public class Aimbot extends Command {
  IO io;
  swerveState state;
  PIDController pid = new PIDController(0.01, 0, 0.01); // TODO tune this
  boolean blue;
  double target;
  boolean auton = false;
  private static final double ANGLE_OFFSET = 0;

  public Aimbot(IO io, swerveState state) {
    this.io = io;
    this.state = state;
  }

  public Aimbot(IO io, swerveState state, boolean auton) {
    this.io = io;
    this.state = state;
    this.auton = auton;
  }

  @Override
  public void initialize() {
    io.chassis.currentState = state;
    blue = DriverStation.getAlliance().get() == Alliance.Blue;
    pid.setTolerance(Math.toRadians(5));
    pid.enableContinuousInput(-Math.PI, Math.PI);
    pid.reset();
  }
  
  @Override
  public void execute() {
    // TODO Get all values
    if (state == swerveState.PASSING) {
      target = blue ? 0 : Math.PI;
    } else if (state == swerveState.SCORING) {
      Translation2d hub = blue ? RobotContainer.BLUE_HUB : RobotContainer.RED_HUB;
      Translation2d diff = hub.minus(io.chassis.getEstimatedPose().getTranslation());
      target = Math.atan2(diff.getY(), diff.getX()) + Math.toRadians(ANGLE_OFFSET);
    }

    SmartDashboard.putNumber("Target", Math.toDegrees(target));

    if(Math.abs((Math.toRadians(io.chassis.getEstimatedRotation())) - target) < Math.toRadians(3)){
      io.chassis.targetRotation = 0.0;
    }
    else{
      if(target - Math.toRadians(io.chassis.getEstimatedRotation()) > 0){
          io.chassis.targetRotation = -0.025;
      }
      else{
        io.chassis.targetRotation = 0.025;
      }
    }

    // TODO find out why PID loop isn't working
    // if(pid.atSetpoint()){
    //   io.chassis.targetRotation = 0.0;
    // }
    // else{
    //   io.chassis.targetRotation = pid.calculate(Math.toRadians(io.chassis.getEstimatedRotation()), target);
    // }
  }

  @Override
  public void end(boolean interrupted) {
    io.chassis.targetRotation = null;
  }

  @Override
  public boolean isFinished() {
    return auton ? Math.abs((io.chassis.getYaw() - 180) - Math.toDegrees(target)) < 7.5 : (io.chassis.currentState != state);
  }
}

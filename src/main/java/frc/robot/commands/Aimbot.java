package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
  PIDController pid = new PIDController(0.2, 0, 0.0001); // TODO tune this
  boolean blue;
  double target;

  public static enum aimbotState {
    DEFAULT, AUTO, NOSTOPAUTO
  }

  aimbotState aimbotMode;

  private static final double ANGLE_OFFSET = Math.toRadians(187);

  public Aimbot(IO io, swerveState state) {
    this(io, state, aimbotState.DEFAULT);
  }

  public Aimbot(IO io, swerveState state, aimbotState aimbotMode) {
    this.io = io;
    this.state = state;
    this.aimbotMode = aimbotMode;
  }

  @Override
  public void initialize() {
    io.chassis.currentState = state;
    blue = DriverStation.getAlliance().get() == Alliance.Blue;
    pid.setTolerance(Math.toRadians(1.5));
    pid.enableContinuousInput(-Math.PI, Math.PI);
    pid.reset();
  }

  @Override
  public void execute() {
    // Math.atan2 returns a range of -pi to pi, this normalizes the estimated
    // rotation.
    double rotation = Math.toRadians(io.chassis.getEstimatedRotation()) - Math.PI;

    if (state == swerveState.SCORINGAIMBOT) {
      Translation2d hub = blue ? RobotContainer.BLUE_HUB : RobotContainer.RED_HUB;
      Translation2d diff = hub.minus(io.chassis.getEstimatedPose().getTranslation());
      target = Math.atan2(diff.getY(), diff.getX()) + ANGLE_OFFSET;
      double output = pid.calculate(rotation, target);

      SmartDashboard.putNumber("Target", Math.toDegrees(target));

      if (aimbotMode == aimbotState.AUTO || aimbotMode == aimbotState.NOSTOPAUTO) {
        io.chassis.drive(new ChassisSpeeds(0, 0, output));
      }
      else {
        io.chassis.targetRotation = pid.atSetpoint() ? 0.0
            : output;
      }

    }
  }

  @Override
  public void end(boolean interrupted) {
    io.chassis.targetRotation = null;
  }

  @Override
  public boolean isFinished() {
    if(aimbotMode == aimbotState.AUTO){
      return pid.atSetpoint();
    }
    else if(aimbotMode == aimbotState.NOSTOPAUTO){
      return io.chassis.currentState != state;
    }
    else{
      return io.chassis.currentState != state;
    }
  }
}

package frc.robot.commands;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.swerveState;
import frc.robot.utility.DistanceShootUtil;
import frc.robot.utility.IO;

public class DistanceShoot extends Command {
  IO io;
  swerveState state;
  boolean blue;
  double target;



  private static final Translation2d BLUE_HUB = new Translation2d(-3.6475, 0); // TODO get positions 
         private static final Translation2d RED_HUB = new Translation2d(3.6423, 0);
//Estimated directly from Limelight Andymark Map (in meters)
  /*
  math in meters instead of feet
  x: distance parallel to floor from shooter to ball
  y: vertical distance to the floor from shooter to target
  a: HoodAngle
  v: Initial total Velocity of ball= RPM*(flywheel radius)*pi/30
  h: height of shooter
  g: gravity = 9.807 m/s^2
  t: time of shot taken (math variable)

  y= v*sin(a)*t - 0.5*g*(t^2)

  x= v*cos(a)*t
  t= x/v/cos(a)
*/

/*
HUB Constants:

math in inches

Height of Hub: 72
Width of hub: 47
Width of funnel: 41.7

Hole short diameter: 23.53
Rim to hole: 9.085
Slant Length of Funnel: 17.9
Height of Funnel: 15.42
Funnel Angle: 1.038 rad, 59.5 deg
Angle of Rim to Hole opposite (side to side): .442 rad, 25.3 deg
 */

/*
 Assumptions about data:
 distance: distance to center of hub in FEET starting at 4 feet
 hoodAngle: Equal to shooting angle
 RPM: RPM of flywheels impacting parallel to ball
 */

 /* 
 Assumed Height of Shooter: 21 inches
 Clearance above rim: 9.58 inches
 Flywheel radius: 4 inches
 */



  DistanceShootUtil[] data;

 double distance;

int point= (int) Math.floor(distance);


double angle = (data[point-4].hoodAngle + data[point-3].hoodAngle)*0.5; // Min. Shooting distance: 4 feet
double RPM = (data[point-4].shooterRPM + data[point-3].shooterRPM)*0.5; //TODO readjust for min. distance for passing








  public DistanceShoot(IO io) {
    this.io = io;
    
    switch (io.chassis.currentState) {
      case SCORING:
        data = new DistanceShootUtil[] {new DistanceShootUtil(0,0,0)};
        break;
      case PASSING:
        data = new DistanceShootUtil[] {new DistanceShootUtil(0,0,0)};
        break;
      default:
        break;
    }

    addRequirements(io.flywheel);
  }

  @Override
  public void initialize() {
    // Set flywheel RPM and hood angle based on distance
      io.flywheel.RPM(RPM);
      io.flywheel.hoodAngle(angle);
  }

  @Override
  public void execute() {
    // TODO Get all values
    if (state == swerveState.PASSING) {
      //target = blue ? Math.toRadians(0) : Math.toRadians(180);
    } else if (state == swerveState.SCORING) {
      Translation2d hub = blue ? BLUE_HUB : RED_HUB;
      Translation2d diff = hub.minus(io.chassis.pose().getTranslation());
      distance = Math.hypot(diff.getY(), diff.getX()) *3.281; //Converted from meters to feet

    // Check if flywheel has hit RPM
      if(io.flywheel.RPM()==RPM){
          io.feeder.speed(.5);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
io.flywheel.stop();
io.feeder.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

package frc.robot.utility;

public class DistanceShootUtil {
    public double distance;
    public double hoodAngle;
    public double shooterRPM;

    public DistanceShootUtil(double distance, double hoodAngle, double shooterRPM) {
        this.distance = distance;
        this.hoodAngle = hoodAngle;
        this.shooterRPM = shooterRPM;
    }

    public DistanceShootUtil(double hoodAngle, double shooterRPM) {
        this.hoodAngle = hoodAngle;
        this.shooterRPM = shooterRPM;
    }
}

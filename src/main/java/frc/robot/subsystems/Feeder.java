package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  public SparkMax feeder;
  public SparkMax agitator;
  public Servo blocker = new Servo(5);
  public int blockerDown = 50;
  public int blockerUp = 120;
  public boolean blocked = true;
  public SparkMaxConfig feederConfig = new SparkMaxConfig();
  public SparkMaxConfig agitatorConfig = new SparkMaxConfig();

  public Feeder(int feederId, int agitatorId) {
    
    agitator = new SparkMax(agitatorId, MotorType.kBrushless);
    agitatorConfig.idleMode(IdleMode.kCoast);
    agitatorConfig.smartCurrentLimit(40);
    agitator.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    feeder = new SparkMax(feederId, MotorType.kBrushless);
    feederConfig.idleMode(IdleMode.kBrake);
    feederConfig.smartCurrentLimit(40);
    feederConfig.closedLoop.pid(0.004, 0, 0.05);
    feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void speed(double speed) {
    feeder.set(speed);
  }

  public void volts(double volts) {
    feeder.setVoltage(volts);
  }

  public void stop() {
    feeder.stopMotor();
  }

  public void block() {
    blocker.setAngle(blockerDown);
  }

  public void unblock() {
    blocker.setAngle(blockerUp);
  }

  public void voltLoop(double volts) {
    feeder.getClosedLoopController().setSetpoint(volts, ControlType.kVoltage);
  }

  public void agitatorSpeed(double speed) {
    agitator.set(speed);
  }

  public void agitatorVolts(double volts) {
    agitator.setVoltage(volts);
  }

  public void stopAgitator() {
    agitator.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Feeder RPM", feeder.getEncoder().getVelocity());
    SmartDashboard.putNumber("Blocker Angle", blocker.getAngle());
    SmartDashboard.putBoolean("Blocked", blocked);
    SmartDashboard.putNumber("Servo Position", blocker.getPosition());
  }
}

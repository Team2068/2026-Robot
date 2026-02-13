package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  public SparkMax flywheel;
  public SparkMaxConfig flywheelConfig;
  public SparkMaxConfig hoodConfig;
  public SparkMax hood;

  public VelocityVoltage control = new VelocityVoltage(0);

  public Flywheel(int shooterId, int hoodId) {
    flywheel = new SparkMax(shooterId, MotorType.kBrushless);
    hood = new SparkMax(hoodId, MotorType.kBrushless);

    flywheelConfig = new SparkMaxConfig();
    flywheelConfig.idleMode(IdleMode.kBrake);
    flywheelConfig.closedLoop.pid(0.2, 0, 0);
    flywheelConfig.smartCurrentLimit(40);
    flywheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    hoodConfig = new SparkMaxConfig();
    hoodConfig.idleMode(IdleMode.kBrake);
    hoodConfig.closedLoop.pid(0.2, 0, 0);
    hoodConfig.smartCurrentLimit(40);
    hood.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void flywheelSpeed(double speed){
    flywheel.set(speed);
  }

  public void flywheelVolts(double volts){
    flywheel.setVoltage(volts);
  }

  public void stopFlywheel(){
    flywheel.stopMotor();
  }

  public void stopHood(){
    hood.stopMotor();
  }

  public void RPM(double rpm){
    rpm = MathUtil.clamp(rpm, 0.0, 6000.0);
    flywheel.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
  }

  public void hoodAngle(double angle){
    hood.getClosedLoopController().setSetpoint(angle / 360.0, ControlType.kPosition);
  }

  public double hoodAngle(){
    return hood.getEncoder().getPosition() * 360;
  }

  public double RPM(){
    return flywheel.getEncoder().getVelocity();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Angle", hoodAngle());
    SmartDashboard.putNumber("Shooter RPM", RPM());
  }
}

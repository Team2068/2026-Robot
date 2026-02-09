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
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  public SparkMax flywheel;
  public SparkMaxConfig config;
  Servo hood;

  public VelocityVoltage control = new VelocityVoltage(0);

  public Flywheel(int shooterId, int hoodId) {
    flywheel = new SparkMax(shooterId, MotorType.kBrushless);
    hood = new Servo(hoodId);

    config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.closedLoop.pid(0.2, 0, 0);
    config.smartCurrentLimit(40);
    flywheel.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void speed(double speed){
    flywheel.set(speed);
  }

  public void volts(double volts){
    flywheel.setVoltage(volts);
  }

  public void stop(){
    flywheel.stopMotor();
  }

  public void RPM(double rpm){
    rpm = MathUtil.clamp(rpm, 0.0, 6000.0);
    flywheel.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
  }

  public void hoodAngle(double degrees){
    hood.setAngle(degrees);
  }

  public double hoodAngle(){
    return hood.getAngle();
  }

  public double RPM(){
    return flywheel.getEncoder().getVelocity();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Angle", hood.getAngle());
    SmartDashboard.putNumber("Shooter RPM", RPM());
  }
}

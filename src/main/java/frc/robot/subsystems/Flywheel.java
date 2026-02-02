// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Flywheel extends SubsystemBase{
  private final TalonFX flywheel;
  private final Servo hood;

  VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  /** Creates a new Subsystem. */
  public Flywheel() {
    flywheel = new TalonFX(16);
    hood = new Servo(0);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kV = 0.05;
    config.Slot0.kP = 0.1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    flywheel.getConfigurator().apply(config);
    }

  public void setVelocity(double RPM){
    flywheel.setControl(velocityVoltage.withVelocity(RPM / 60.0));
  }
  

  public void setHoodPosition(double position) {
    hood.setPosition(position);
  }


  public void setSpeed(double speed) {
    flywheel.set(speed);
  }

  // create a set voltage
  public void setVoltage(double volts){
    flywheel.setVoltage(volts);
  }

  public void stop() {
    flywheel.stopMotor();
  }

  public double getFlywheelRPM() {
    return flywheel.getVelocity().getValueAsDouble() * 60.0;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // print rpm to smart dashboard
    SmartDashboard.putNumber("Hood Angle", hood.get());
    // print hood position to smart dashboard
    SmartDashboard.putNumber("Flywheel RPM", getFlywheelRPM());
    }

}

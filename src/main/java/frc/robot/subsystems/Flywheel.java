// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


//import java.lang.invoke.ClassSpecializer.SpeciesData;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Flywheel extends SubsystemBase{
  SparkMaxConfig config;
  SparkMax motor;
  SparkMaxConfig config2;
  SparkMax motor2;
  /** Creates a new Subsystem. */
  public Flywheel() {
    config = new SparkMaxConfig();
    config2 = new SparkMaxConfig();
    motor = new SparkMax(2, MotorType.kBrushless);
    motor2 = new SparkMax(3, MotorType.kBrushless);

    config.idleMode(IdleMode.kBrake);
    motor.configure(config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config2
      .inverted(true)
    .idleMode(IdleMode.kBrake);
    motor2.configure(config2,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }


  public void setSpeed(double speed1, double speed2) {
    motor.set(speed1);
    motor2.set(speed2);
  }

  public void setVoltage(double voltage1, double voltage2) {
    motor.setVoltage(voltage1);
    motor2.setVoltage(voltage2);
  }

  public void stop() {
    motor.stopMotor();
    motor2.stopMotor();
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  

}

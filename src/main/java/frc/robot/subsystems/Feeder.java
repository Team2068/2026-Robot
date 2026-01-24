// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  private SparkMax feeder;

  private SparkMaxConfig feederConfig = new SparkMaxConfig();
  public Feeder(int id) {
feeder  = new SparkMax(id, MotorType.kBrushless);
    feederConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void speed(double speed) {
    feeder.set(speed);
  }

  public void volts(double voltage) {
    feeder.setVoltage(voltage);
  }
  
  public void stop() {
    feeder.stopMotor();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

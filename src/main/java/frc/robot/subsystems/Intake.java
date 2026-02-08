package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public SparkMax intake;
  public SparkMaxConfig config;

  private boolean active = false;


  public Intake(int id) {
    intake = new SparkMax(id, MotorType.kBrushed);
    
    config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    
    intake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void speed(double speed){
    intake.set(speed);
    active = speed != 0.0;
  }

  public void volts(double volts){
    intake.setVoltage(volts);
    active = volts != 0.0;
  }

  public void stop(){
    intake.stopMotor();
    active = false;
  }

  public void intake(){
    if(!active)
      speed(-1);
    else
      stop();
  }

  public boolean active(){
    return active;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Active", active);
    SmartDashboard.putNumber("Intake Speed", intake.get());
  }
}

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  public SparkMax feeder;
  public DigitalInput beambreak;
  public SparkMaxConfig config = new SparkMaxConfig();

  public Feeder(int feederId, int beambreakId) {
    feeder = new SparkMax(feederId, MotorType.kBrushless);
    beambreak = new DigitalInput(beambreakId);
    
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(20);
    feeder.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void speed(double speed){
    feeder.set(speed);
  }

  public void volts(double volts){
    feeder.setVoltage(volts);
  }

  public void stop(){
    feeder.stopMotor();
  }

  public boolean supplied(){
    return beambreak.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Supplied", supplied());
  }
}

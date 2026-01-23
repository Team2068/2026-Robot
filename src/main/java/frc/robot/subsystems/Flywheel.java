package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  public TalonFX flywheel;
  public TalonFXConfiguration config;
  Servo hood;

  public VelocityVoltage control = new VelocityVoltage(0);

  public Flywheel(int shooterId, int hoodId) {
    flywheel = new TalonFX(shooterId, "rio");
    hood = new Servo(hoodId);

    config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Slot0.kP = 0.0;
    config.Slot0.kV = 0.0;

    flywheel.getConfigurator().apply(config);
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

    flywheel.setControl(control.withVelocity(rpm / 60.0));
  }

  public void hoodAngle(double degrees){
    hood.setAngle(degrees);
  }

  public double RPM(){
    return flywheel.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Angle", hood.getAngle());
    SmartDashboard.putNumber("Shooter RPM", RPM());
  }
}

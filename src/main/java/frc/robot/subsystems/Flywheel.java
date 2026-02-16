package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
  public TalonFX flywheel;
  public TalonFXConfiguration config;
  public SparkMaxConfig hoodConfig;
  public SparkMax hood;

  public VelocityVoltage control = new VelocityVoltage(0);

  public Flywheel(int shooterId, int hoodId) {
    flywheel = new TalonFX(shooterId, "rio");
    hood = new SparkMax(hoodId, MotorType.kBrushless);

    config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Slot0.kP = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.0;

    flywheel.getConfigurator().apply(config);

    hoodConfig = new SparkMaxConfig();
    hoodConfig.idleMode(IdleMode.kBrake);
    hoodConfig.closedLoop.pid(0.2, 0, 0);
    hoodConfig.smartCurrentLimit(40);
    hood.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void flywheelSpeed(double speed) {
    flywheel.set(speed);
  }

  public void flywheelVolts(double volts) {
    flywheel.setVoltage(volts);
  }

  public void stopFlywheel() {
    flywheel.stopMotor();
  }

  public void stopHood() {
    hood.stopMotor();
  }

  public void hoodSpeed(double speed){
    hood.set(speed);
  }

  public void RPM(double rpm) {
    rpm = MathUtil.clamp(rpm, 0.0, 6000.0);
    flywheel.setControl(control.withVelocity(rpm / 60.0));
  }

  public void hoodAngle(double angle) {
    hood.getClosedLoopController().setSetpoint(angle, ControlType.kPosition);
  }

  public double hoodAngle() {
    return hood.getEncoder().getPosition();
  }

  public double RPM() {
    return flywheel.getVelocity().getValueAsDouble() * 60;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Angle", hoodAngle());
    SmartDashboard.putNumber("Shooter RPM", RPM());
  }
}

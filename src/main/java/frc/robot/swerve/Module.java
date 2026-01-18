package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class Module {
    public final TalonFX drive;
    public final TalonFX steer;
    public final Swerve.Encoder encoder;

    double desiredAngle;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double STEER_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

    public Module(ShuffleboardLayout tab, int driveID, int steerID, int encoderID, boolean heliumEncoder) {
        drive = new TalonFX(driveID, "rio");
        steer = new TalonFX(steerID, "rio");
        encoder = (heliumEncoder) ? new Swerve.Canand(encoderID) : new Swerve.Cancoder(encoderID);

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        steerConfig.CurrentLimits.StatorCurrentLimit = 80;
        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfig.CurrentLimits.SupplyCurrentLimit = 20;
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        steerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // TODO Tune PID Values
        steerConfig.Slot0.kP = 0.2;
        steerConfig.Slot0.kI = 0.0;
        steerConfig.Slot0.kD = 0.0;
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

        // replaces our conversion factors
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        steerConfig.Feedback.RotorToSensorRatio = STEER_REDUCTION;
        
        steer.getConfigurator().apply(steerConfig);
        // set position in phoenix returns mechanism rotations so converts encoder angle to rotations.
        steer.setPosition(angle() / Swerve.PI2);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        driveConfig.CurrentLimits.StatorCurrentLimit = 80;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        driveConfig.CurrentLimits.SupplyCurrentLimit = 20;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        drive.getConfigurator().apply(driveConfig);

        tab.addDouble("Absolute Angle", () -> Math.toDegrees(angle()));
        tab.addDouble("Current Angle", () -> steer.getPosition().getValueAsDouble() * Swerve.PI2);
        tab.addDouble("Angle Difference", () -> Math.toDegrees(angle() - (steer.getPosition().getValueAsDouble() * Swerve.PI2)));
        tab.addDouble("Target Angle", () -> Math.toDegrees(desiredAngle));
        tab.addBoolean("Active", encoder::connected);
    }

    public void resetDrivePosition() {
        drive.setPosition(0.0);
    }

    public void syncEncoders() {
        steer.setPosition(encoder.angle() / Swerve.PI2);
    }

    public void zeroAbsolute() {
        encoder.zero();
    }

    public double drivePosition() {
        return drive.getPosition().getValueAsDouble() * .632 * WHEEL_DIAMETER;
    }

    public LinearVelocity velocity() {
        return MetersPerSecond.of(drive.getVelocity().getValueAsDouble() * Swerve.PI2 * .632 * WHEEL_DIAMETER);
    }

    public Voltage voltage(){
        return drive.getMotorVoltage().getValue();
    }

    public double angle() {
        return encoder.angle();
    }

    public AngularVelocity steerVelocity(){
        return encoder.velocity();
    }

    public Voltage steerVoltage(){
        return steer.getSupplyVoltage().getValue();
    }

    public void stop() {
        drive.stopMotor();
        steer.stopMotor();
    }

    public void set(double driveVolts, double targetAngle) {
        double normalized = MathUtil.inputModulus(targetAngle, 0, Swerve.PI2);
        syncEncoders();
        drive.set(driveVolts);
        steer.setControl(new PositionVoltage(0).withPosition(normalized / Swerve.PI2));
    }

    public void setSteer(double steerVolts){
        syncEncoders();
        drive.set(0);
        steer.set(steerVolts);
    }
}
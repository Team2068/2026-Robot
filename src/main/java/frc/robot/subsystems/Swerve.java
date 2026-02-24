package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.swerve.Module;
import frc.robot.swerve.Swerve.Constants;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.Util;

public class Swerve extends SubsystemBase {

    public boolean field_oritented = true;
    private final SwerveDriveKinematics kinematics;

    public final Pigeon2 pigeon2 = new Pigeon2(Constants.PIGEON_ID, "Swerve");

    StructArrayPublisher<SwerveModuleState> current_states = Util.table
            .getStructArrayTopic("Current Module States", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> target_states = Util.table
            .getStructArrayTopic("Target Module States", SwerveModuleState.struct).publish();
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getTable("Debug")
            .getStructTopic("Current pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> estimatedPublisher = NetworkTableInstance.getDefault().getTable("Debug")
            .getStructTopic("Estimated pose", Pose2d.struct).publish();
        StructPublisher<Pose2d> estimatedPublisherWithYaw = NetworkTableInstance.getDefault().getTable("Debug")
            .getStructTopic("Estimated pose", Pose2d.struct).publish();

    SwerveDrivePoseEstimator estimator;
    SwerveDrivePoseEstimator estimatorWithYaw;

    final SwerveDriveOdometry odometry;
    final Module[] modules = new Module[4];
    ChassisSpeeds speeds = new ChassisSpeeds();
    public final Constants constants = new Constants();
    Timer timer = new Timer();

    public boolean active = true;

    public static enum swerveState {
        DEFAULT, SCORING, PASSING
    }

    public Double targetRotation = null;

    public swerveState currentState = swerveState.DEFAULT;

    public Swerve() {
        kinematics = new SwerveDriveKinematics(
                createTranslation(constants.TRACKWIDTH / 2.0, constants.WHEELBASE / 2.0),
                createTranslation(constants.TRACKWIDTH / 2.0, -constants.WHEELBASE / 2.0),
                createTranslation(-constants.TRACKWIDTH / 2.0, constants.WHEELBASE / 2.0),
                createTranslation(-constants.TRACKWIDTH / 2.0, -constants.WHEELBASE / 2.0));

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new Module(
                    tab.getLayout(Constants.LAYOUT_TITLE[i], BuiltInLayouts.kList)
                            .withSize(2, 4)
                            .withPosition(i * 2, 0),
                    Constants.DRIVE_ID[i],
                    Constants.STEER_ID[i],
                    Constants.ENCODER_ID[i],
                    constants.heliumEncoders);
        }

        odometry = new SwerveDriveOdometry(kinematics, rotation(), modulePositions(),
                new Pose2d(0, 0, new Rotation2d()));

        estimator = new SwerveDrivePoseEstimator(kinematics, rotation(), modulePositions(), odometry.getPoseMeters());
        estimatorWithYaw = new SwerveDrivePoseEstimator(kinematics, rotation(), modulePositions(), odometry.getPoseMeters());

        AutoBuilder.configure(
                this::getEstimatedPose, // See how using pose estimation affects auton.
                this::resetOdometry,
                this::getSpeeds,
                (speeds, feedforwards) -> drive(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(1.7, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(0.0, 0.0, 0.0) // Rotation PID constants
                ),
                constants.autoConfig, // The robot configuration
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this);

        timer.start();
    }

    private Translation2d createTranslation(double x, double y) {
        return new Translation2d(x, y);
    }

    public void resetAngle() {
        pigeon2.setYaw(0);
    }

    public Rotation2d rotation() {
        double rotation = pigeon2.getYaw().getValueAsDouble() % 360;
        rotation += (rotation < 0) ? 360 : 0;
        return new Rotation2d(Degree.of(rotation));
    }

    public void adjustRotation() {
        pigeon2.setYaw((rotation().getDegrees() + 180) % 360);
    }

    public void drive(ChassisSpeeds speeds) {
        this.speeds = speeds;
    }

    public void stop() {
        speeds = new ChassisSpeeds();
    }

    public double distance(Pose2d reference_point) {
        return pose().getTranslation().getDistance(reference_point.getTranslation());
    }

    public SwerveModulePosition[] modulePositions() {
        SwerveModulePosition[] pos = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++)
            pos[i] = new SwerveModulePosition(modules[i].drivePosition(), Rotation2d.fromRadians(modules[i].angle()));
        return pos;
    }

    public double getYaw() {
        return (pigeon2.getYaw().getValueAsDouble() % 360 + 360) % 360;
    }

    public Pose2d pose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry() {
        resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(0)));
    }

    public void resetOdometry(Pose2d pose) {
        pigeon2.setYaw(pose.getRotation().getDegrees());
        resetModulePositions();

        odometry.resetPosition(rotation(), modulePositions(), pose);
    }

    public void resetModulePositions() {
        for (Module mod : modules)
            mod.resetDrivePosition();
    }

    public void syncEncoders() {
        for (Module mod : modules)
            mod.syncEncoders();
    }

    public void zeroAbsolute() {
        for (Module mod : modules)
            mod.zeroAbsolute();
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(moduleStates(modules));
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_VELOCITY);
        for (int i = 0; i < modules.length; i++) {
            states[i].optimize(new Rotation2d(modules[i].angle()));
            modules[i].set((states[i].speedMetersPerSecond / Constants.MAX_VELOCITY) * .8,
                    states[i].angle.getRadians());
        }
    }

    public SwerveModuleState[] moduleStates(Module[] modules) {
        SwerveModuleState[] state = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++)
            state[i] = new SwerveModuleState(modules[i].velocity(), new Rotation2d(modules[i].angle()));
        return state;
    }

    final MutDistance[] distance = { Meters.mutable(0), Meters.mutable(0), Meters.mutable(0), Meters.mutable(0) };
    final MutAngle[] angle = { Radians.mutable(0), Radians.mutable(0), Radians.mutable(0), Radians.mutable(0) };

    public final SysIdRoutine driveRoutine = new SysIdRoutine(new Config(
            null,
            Volts.of(3),
            // Seconds.of(5),
            null,
            null),
            new SysIdRoutine.Mechanism(voltage -> {
                for (Module mod : modules)
                    mod.set(voltage.magnitude(), 0);
            }, log -> {
                log.motor(Constants.LAYOUT_TITLE[0] + " [Drive]")
                        .voltage(modules[0].voltage())
                        .linearPosition(distance[0].mut_replace(modules[0].drivePosition(), Meters))
                        .linearVelocity(modules[0].velocity())
                        .linearAcceleration(pigeon2.getAccelerationX().getValue()); // NOTE: This is due to our gyro
                                                                                    // being mounted 90 degrees off
                                                                                    // centre
            }, this));

    public final SysIdRoutine steerRoutine = new SysIdRoutine(new Config(
            null,
            Volts.of(2),
            null,
            null),
            new SysIdRoutine.Mechanism(voltage -> {
                speeds = new ChassisSpeeds(0, 0, (voltage.magnitude() / 16.0) * Constants.MAX_ANGULAR_VELOCITY);
            }, log -> {
                log.motor("Chassis")
                        .voltage(modules[0].steerVoltage())
                        .angularPosition(pigeon2.getYaw().getValue())
                        .angularVelocity(pigeon2.getAngularVelocityYWorld().getValue());
            }, this));

    public double getRoll() {
        return pigeon2.getRoll().getValueAsDouble();
    }

    public void toggle() {
        active = !active;

        for (Module mod : modules)
            mod.stop();
    }

    public void estimatePose() {
        estimator.update(rotation(), modulePositions());
        estimatorWithYaw.update(rotation(), modulePositions());
        LimelightHelpers.SetRobotOrientation("limelight-main", pigeon2.getYaw().getValueAsDouble(), 0,
                pigeon2.getPitch().getValueAsDouble(), 0, pigeon2.getRoll().getValueAsDouble(), 0);
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-main");
        boolean doRejectUpdate = false;

        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > .30) {
                doRejectUpdate = true;
            }
            if (mt1.rawFiducials[0].distToCamera > 3) {
                doRejectUpdate = true;
            }
        }
        if (mt1.tagCount == 0) {
            doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
            estimator.setVisionMeasurementStdDevs(VecBuilder.fill(.05, .07, 9999999));
            estimatorWithYaw.setVisionMeasurementStdDevs(VecBuilder.fill(.05, .07, 0.5));
            estimator.addVisionMeasurement(
                    mt1.pose,
                    mt1.timestampSeconds);
            estimatedPublisher.set(estimator.getEstimatedPosition());

            estimatorWithYaw.addVisionMeasurement(
                    mt1.pose,
                    mt1.timestampSeconds);
            estimatedPublisherWithYaw.set(estimatorWithYaw.getEstimatedPosition());
        }
    }

    public Pose2d getEstimatedPose() {
        return estimator.getEstimatedPosition();
    }

    public Pose2d getEstimatedPoseWithYaw() {
        return estimatorWithYaw.getEstimatedPosition();
    }

    public double getEstimatedRotation(){
        return getEstimatedPoseWithYaw().getRotation().getDegrees();
    }

    public void periodic() {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        if (active && speeds != new ChassisSpeeds())
            setModuleStates(states);

        current_states.set(moduleStates(modules));
        target_states.set(states);

        Pose2d pose = odometry.update(rotation(), modulePositions());
        estimatePose();
        posePublisher.set(pose);
        estimatedPublisher.set(estimator.getEstimatedPosition());
        estimatedPublisherWithYaw.set(estimatorWithYaw.getEstimatedPosition());

        SmartDashboard.putNumber("Timer", timer.get());

        if ((int) (timer.get()) % 10 == 0) {
            syncEncoders();
        }

        SmartDashboard.putNumber("X position", pose.getX());
        SmartDashboard.putNumber("Y position", pose.getY());
        SmartDashboard.putNumber("Estimated Rotation", getEstimatedRotation());

        SmartDashboard.putNumber("Odometry rotation", rotation().getDegrees());
        SmartDashboard.putNumber("Pigeon Yaw", getYaw());
        SmartDashboard.putNumber("Pigeon Pitch",
                pigeon2.getPitch().getValueAsDouble());
        SmartDashboard.putNumber("Pigeon Roll",
                pigeon2.getRoll().getValueAsDouble());

        SmartDashboard.putString("Drive Mode", (field_oritented) ? "Field-Oriented" : "Robot-Oriented");

        if (targetRotation != null)
            SmartDashboard.putNumber("Target Rotation", targetRotation * 360);
        else
            SmartDashboard.putNumber("Target Rotation", 0.0);

        SmartDashboard.putString("Robot State", currentState.toString());
    }
}
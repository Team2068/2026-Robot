package frc.robot.utility;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class IO extends SubsystemBase {
        public final Swerve chassis = new Swerve();
        public final Intake intake = new Intake(15);
        public final Flywheel flywheel = new Flywheel(16, 0);
        public final Feeder feeder = new Feeder(17, 0);

        public IO() {
           DriverStation.silenceJoystickConnectionWarning(true);
        }

        @Override
        public void periodic() {}
}
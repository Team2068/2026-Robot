// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve.swerveState;
import frc.robot.utility.IO;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DistanceShootAgitate extends SequentialCommandGroup {
  /** Creates a new DistanceShootAgitate. */
  public DistanceShootAgitate(IO io) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new Aimbot(io, swerveState.SCORINGAIMBOT, true), new DistanceShoot(io, true), new AgitateChassis(io), new WaitCommand(0.3), new Aimbot(io, swerveState.SCORINGAIMBOT, true), new DistanceShoot(io, true));
  }
}

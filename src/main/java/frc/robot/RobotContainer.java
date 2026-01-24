// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.utility.IO;
import frc.robot.utility.AutomatedController;
import frc.robot.utility.Util;
import frc.robot.commands.DefaultDrive;

public class RobotContainer {
  public IO io = new IO();
  public final AutomatedController main;

  // private final SendableChooser<Command> auto_selector;
  // Command current_auto = new PrintCommand("");
  final SendableChooser<Integer> driver_selector = new SendableChooser<Integer>();

  public RobotContainer() {
    main = new AutomatedController(0, io);
    CanandEventLoop.getInstance();

    // configureAuton();

    // auto_selector = AutoBuilder.buildAutoChooser();
    // auto_selector.onChange((command) -> {
    //   current_auto = command;
    // });

    // SmartDashboard.putData("Autos", auto_selector);
    // SmartDashboard.putData("Run Test Auto", Util.Do(current_auto::schedule));

    SmartDashboard.putData("Main-Controller Mode", main.selector);

    SmartDashboard.putData("Driver", driver_selector);
    io.chassis.setDefaultCommand(new DefaultDrive(io, main.controller));
  }

  public void configureAuton() {
    // Create auton commands
  }

  // public Command getAutonomousCommand() {
  //   return auto_selector.getSelected();
  // }
}

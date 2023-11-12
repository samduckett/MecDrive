// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.Constants.Controls;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final DrivetrainBase drivetrain = new MecDrivetrain();

  @Override
  public void robotInit() {
    drivetrain.setDefaultCommand(new TeleopDrivetrain(drivetrain));

    registerCommand();
    // autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Zero Gyro", new InstantCommand(drivetrain::zeroGyro));
    // autoChooser.addOption("TestAuto1", new PathPlannerAuto("New Auto"));
    SmartDashboard.putData("Auto Chooser", autoChooser);

    Controls.driver.A.onTrue(new InstantCommand(drivetrain::zeroGyro));
    // Controls.operator.B.whileTrue(new InstantCommand());
  }

  public void registerCommand() {
    NamedCommands.registerCommand("zeroGyro", new InstantCommand(drivetrain::zeroGyro));
    NamedCommands.registerCommand("PlacePiceHigh", new InstantCommand(drivetrain::zeroGyro));
    NamedCommands.registerCommand("PlacePiceMid", new InstantCommand(drivetrain::zeroGyro));
    NamedCommands.registerCommand("PlacePiceLow", new InstantCommand(drivetrain::zeroGyro));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = autoChooser.getSelected();
    if (autonomousCommand != null)
      autonomousCommand.schedule();
    else
      System.err.print("fix your code");
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null)
      autonomousCommand.cancel();
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
  private Command m_autonomousCommand;
  private SendableChooser<Command> autoChooser;

  private final DrivetrainBase drivetrain = new MecDrivetrain();

  @Override
  public void robotInit() {
    drivetrain.setDefaultCommand(new TeleopDrivetrain(drivetrain));

    NamedCommands.registerCommand("zeroGyro", new InstantCommand(drivetrain::zeroGyro));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    Controls.driver.A.onTrue(new InstantCommand(drivetrain::zeroGyro));
    // Controls.operator.B.whileTrue(new InstantCommand());
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
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    autoChooser.getSelected().schedule();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}

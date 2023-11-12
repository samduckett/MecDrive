package frc.robot.commands;

import frc.robot.Constants.Controls;
import frc.robot.subsystems.DrivetrainBase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrivetrain extends CommandBase {
	private DrivetrainBase drivetrain;

	public TeleopDrivetrain(DrivetrainBase drivetrain) {
		this.drivetrain = drivetrain;
		addRequirements(drivetrain);
	}

	@Override
	public void execute() {
		drivetrain.drive(
				new Translation2d(
						Controls.driver.LS_Y.get(),
						Controls.driver.LS_X.get()).times(Controls.driver.LT_S.get() > 0.5 ? 0.25 : 1)
						.times(Controls.driver.A.getAsBoolean() ? 1 / drivetrain.maxSpeed : 1),
				(Controls.driver.LT_S.get() > 0.5 ? Controls.driver.LT_S.get() * .25 : Controls.driver.LT_S.get())
						* drivetrain.maxAngularVelocity,
				!Controls.driver.LB.getAsBoolean());

		SmartDashboard.putNumber("JoyX", Controls.driver.LS_Y.get());
		SmartDashboard.putNumber("JoyX", Controls.driver.LS_X.get());
		SmartDashboard.putNumber("JoyX", Controls.driver.LT_S.get());

	}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.commands.AutonomousCommand;
import frc.robot.drive.commands.TeleOpCommand;

public class RobotContainer {
	private SwerveDrive sd;

	public RobotContainer() {
		configureBindings();
		sd = new SwerveDrive(new Pose2d());
	}

	private void configureBindings() {}

	public Command getAutonomousCommand() {
		return new AutonomousCommand(sd, new Pose2d(new Translation2d(3, -1), new Rotation2d()));
	}

	public Command getTeleOpCommand() {
		return new TeleOpCommand(new CommandXboxController(RobotConstants.controllerPort), sd);
	}
}

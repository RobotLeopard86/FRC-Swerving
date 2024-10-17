// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.arm.ArmCommand;
import frc.robot.arm.ArmConstants;
import frc.robot.arm.ArmSystem;
import frc.robot.arm.commands.ArmDownCommand;
import frc.robot.arm.commands.ArmToPositionCommand;
import frc.robot.arm.commands.ArmUpCommand;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.commands.AutonomousCommand;
import frc.robot.drive.commands.TeleOpCommand;

public class RobotContainer {
	private SwerveDrive sd;
	private ArmSystem arm;
	private CommandXboxController xbox;

	public RobotContainer() {
		configureBindings();
		sd = new SwerveDrive(new Pose2d());
		sd.setDefaultCommand(getTeleOpCommand());
		arm = new ArmSystem();
		xbox = new CommandXboxController(RobotConstants.controllerPort);
	}

	private void configureBindings() {
		xbox.pov(0).onTrue(new ArmUpCommand(arm));
		xbox.pov(180).onTrue(new ArmDownCommand(arm));
		xbox.b().onTrue(new ArmToPositionCommand(arm, Rotation2d.fromDegrees(ArmConstants.armStowDegrees)));
		xbox.leftTrigger().onTrue(new ArmToPositionCommand(arm, Rotation2d.fromDegrees(ArmConstants.armSpeakerShootDegrees)));
		xbox.rightTrigger().onTrue(new ArmToPositionCommand(arm, Rotation2d.fromDegrees(ArmConstants.armIntakeDegrees)));
		xbox.leftBumper().onTrue(new ArmToPositionCommand(arm, Rotation2d.fromDegrees(ArmConstants.armAmpShootDegrees)));
	}

	public Command getAutonomousCommand() {
		return new AutonomousCommand(sd, new Pose2d(new Translation2d(3, -1), new Rotation2d()));
	}

	public Command getTeleOpCommand() {
		return new TeleOpCommand(xbox, sd);
	}
}

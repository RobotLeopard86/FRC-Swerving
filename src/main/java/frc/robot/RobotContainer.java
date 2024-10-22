// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.arm.ArmConstants;
import frc.robot.arm.ArmSystem;
import frc.robot.arm.commands.ArmRelativeMoveCommand;
import frc.robot.arm.commands.ArmToPositionCommand;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.commands.AutonomousCommand;
import frc.robot.drive.commands.TeleOpCommand;

public class RobotContainer {
	private SwerveDrive drive;
	private ArmSystem arm;
	private CommandXboxController xbox;

	public RobotContainer() {
		xbox = new CommandXboxController(RobotConstants.CONTROLLER_PORT);
		drive = new SwerveDrive(new Pose2d());
		drive.setDefaultCommand(getTeleOpCommand());
		arm = new ArmSystem();
		configureBindings();
	}

	private void configureBindings() {
		xbox.pov(0).onTrue(new ArmRelativeMoveCommand(arm, false));
		xbox.pov(180).onTrue(new ArmRelativeMoveCommand(arm, true));
		xbox.b().onTrue(new ArmToPositionCommand(arm, Rotation2d.fromDegrees(ArmConstants.ARM_STOW_DEGREES)));
		xbox.leftTrigger().onTrue(new ArmToPositionCommand(arm, Rotation2d.fromDegrees(ArmConstants.ARM_SPEAKER_SHOOT_DEGREES)));
		xbox.rightTrigger().onTrue(new ArmToPositionCommand(arm, Rotation2d.fromDegrees(ArmConstants.ARM_INTAKE_DEGREES)));
		xbox.leftBumper().onTrue(new ArmToPositionCommand(arm, Rotation2d.fromDegrees(ArmConstants.ARM_AMP_SHOOT_DEGREES)));
	}

	public Command getAutonomousCommand() {
		return new AutonomousCommand(drive, new Pose2d(new Translation2d(3, -1), new Rotation2d()));
	}

	public Command getTeleOpCommand() {
		return new TeleOpCommand(xbox, drive);
	}
}

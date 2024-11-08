package frc.robot.drive.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drive.DriveConstants;
import frc.robot.drive.SwerveDrive;

public class TeleOpCommand extends Command {

    private CommandXboxController xbox;
    private SwerveDrive drive;

    public TeleOpCommand(CommandXboxController controller, SwerveDrive drive) {
        xbox = controller;
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = new ChassisSpeeds(
                MathUtil.applyDeadband(-xbox.getLeftY(), DriveConstants.CONTROLLER_DEADBAND),
                MathUtil.applyDeadband(-xbox.getLeftX(), DriveConstants.CONTROLLER_DEADBAND),
                MathUtil.applyDeadband(-xbox.getRightX(), DriveConstants.CONTROLLER_DEADBAND));
        drive.setRobotSpeeds(speeds);
    }

    @Override
    public void initialize() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

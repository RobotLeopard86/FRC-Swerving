package frc.robot.drive.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
        drive.setRobotSpeeds(new ChassisSpeeds(-xbox.getLeftY(), -xbox.getLeftX(), -xbox.getRightX()));
    }

    @Override
    public void initialize() {}
    
    @Override
    public boolean isFinished() {
        return false;
    }
}

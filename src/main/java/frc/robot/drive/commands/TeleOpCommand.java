package frc.robot.drive.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drive.SwerveDrive;

public class TeleOpCommand extends Command {

    CommandXboxController xbox;
    SwerveDrive drive;

    public TeleOpCommand(CommandXboxController controller, SwerveDrive drive) {
        xbox = controller;
        this.drive = drive;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        super.end(interrupted);
    }

    @Override
    public void execute() {
        drive.setRobotSpeeds(new ChassisSpeeds(-xbox.getLeftY(), -xbox.getLeftX(), -xbox.getRightX()));
        super.execute();
    }

    @Override
    public void initialize() {
        super.initialize();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}

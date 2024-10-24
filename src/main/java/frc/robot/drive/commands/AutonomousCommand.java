package frc.robot.drive.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drive.SwerveDrive;

public class AutonomousCommand extends Command {
    SwerveDrive drive;
    Pose2d target;
    PIDController xPidController, yPidController, turnPidController;

    public AutonomousCommand(SwerveDrive drive, Pose2d targetPose) {
        this.drive = drive;
        target = targetPose;
        xPidController = new PIDController(0, 0, 0);
        yPidController = new PIDController(0, 0, 0);
        turnPidController = new PIDController(0, 0, 0);
        addRequirements(drive);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public void execute() {
        Pose2d curPose = drive.getPose();
        drive.setRobotSpeeds(new ChassisSpeeds(
            xPidController.calculate(curPose.getX(), target.getX()), 
            yPidController.calculate(curPose.getY(), target.getY()), 
            turnPidController.calculate(curPose.getRotation().getRadians(), curPose.getRotation().getRadians())
        ));
    }

    @Override
    public void initialize() {}
    
    @Override
    public boolean isFinished() {
        return drive.getPose() == target;
    }
    
}

package frc.robot.drive.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drive.SwerveDrive;

public class AutonomousCommand extends Command {
    SwerveDrive drive;
    Pose2d target;
    PIDController xc, yc, tc;

    public AutonomousCommand(SwerveDrive drive, Pose2d targetPose) {
        this.drive = drive;
        target = targetPose;
        xc = new PIDController(0, 0, 0);
        yc = new PIDController(0, 0, 0);
        tc = new PIDController(0, 0, 0);
        this.addRequirements(drive);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        super.end(interrupted);
    }

    @Override
    public void execute() {
        Pose2d curPose = drive.getPose();
        drive.setRobotSpeeds(new ChassisSpeeds(xc.calculate(curPose.getX(), target.getX()), yc.calculate(curPose.getY(), target.getY()), tc.calculate(curPose.getRotation().getRadians(), curPose.getRotation().getRadians())));
        super.execute();
    }

    @Override
    public void initialize() {
        super.initialize();
    }
    
    @Override
    public boolean isFinished() {
        return drive.getPose() == target;
    }
    
}

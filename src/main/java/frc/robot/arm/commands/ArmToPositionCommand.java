package frc.robot.arm.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.arm.ArmSystem;

public class ArmToPositionCommand extends Command {
    private ArmSystem arm;
    private Rotation2d target;

    public ArmToPositionCommand(ArmSystem arm, Rotation2d target) {
        this.arm = arm;
        this.target = target;
        this.addRequirements(arm);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public void execute() {
        arm.setCurrentAngle(target);
        super.execute();
    }

    @Override
    public void initialize() {
        super.initialize();
    }
    
    @Override
    public boolean isFinished() {
        return arm.getCurrentAngle() == target;
    }
}

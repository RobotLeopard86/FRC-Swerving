package frc.robot.arm.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.arm.ArmConstants;
import frc.robot.arm.ArmSystem;

public class ArmDownCommand extends Command {
    private ArmSystem arm;
    private Rotation2d target;

    public ArmDownCommand(ArmSystem arm) {
        this.arm = arm;
        this.addRequirements(arm);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public void execute() {
        Rotation2d a = arm.getCurrentAngle().minus(Rotation2d.fromDegrees(ArmConstants.armMoveDegrees));
        a = Rotation2d.fromDegrees(Math.min(Math.max(a.getDegrees(), ArmConstants.minArmDegrees), ArmConstants.maxArmDegrees));
        target = a;
        arm.setCurrentAngle(a);
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

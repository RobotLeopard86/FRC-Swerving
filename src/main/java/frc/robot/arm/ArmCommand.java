package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ArmCommand extends Command {
    private CommandXboxController xbox;
    private ArmSystem arm;

    public ArmCommand(CommandXboxController controller, ArmSystem arm) {
        xbox = controller;
        this.arm = arm;
        this.addRequirements(arm);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public void execute() {
        Rotation2d a = arm.getCurrentAngle();

        if(xbox.povUp().getAsBoolean()) a = a.plus(Rotation2d.fromDegrees(ArmConstants.armMoveDegrees));
        if(xbox.povDown().getAsBoolean()) a = a.minus(Rotation2d.fromDegrees(ArmConstants.armMoveDegrees));
        if(xbox.b().getAsBoolean()) a = Rotation2d.fromDegrees(ArmConstants.armStowDegrees);
        if(xbox.leftTrigger().getAsBoolean()) a = Rotation2d.fromDegrees(ArmConstants.armSpeakerShootDegrees);
        if(xbox.rightTrigger().getAsBoolean()) a = Rotation2d.fromDegrees(ArmConstants.armIntakeDegrees);
        if(xbox.leftBumper().getAsBoolean()) a = Rotation2d.fromDegrees(ArmConstants.armAmpShootDegrees);

        a = Rotation2d.fromDegrees(Math.min(Math.max(a.getDegrees(), ArmConstants.minArmDegrees), ArmConstants.maxArmDegrees));
        arm.setCurrentAngle(a);
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

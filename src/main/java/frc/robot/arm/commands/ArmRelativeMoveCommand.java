package frc.robot.arm.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.arm.ArmConstants;
import frc.robot.arm.ArmSystem;

public class ArmRelativeMoveCommand extends ArmToPositionCommand {
    private boolean goesDown;

    public ArmRelativeMoveCommand(ArmSystem arm, boolean goesDown) {
        super(arm, Rotation2d.fromDegrees(0));
        this.goesDown = goesDown;
    }

    //Note: Since this extends ArmToPositionCommand, the definitions for init(), isFinished(), and end() are there 
    //as they don't have any unique behavior between these two classes

    @Override
    public void execute() {
        target = (goesDown
            ? arm.getCurrentAngle().minus(Rotation2d.fromDegrees(ArmConstants.ARM_MOVE_DEGREES)) 
            : arm.getCurrentAngle().plus(Rotation2d.fromDegrees(ArmConstants.ARM_MOVE_DEGREES))
        );
        
        //Now we have to clamp it... JDK 21 does this for us but we're not on 21 so *angry noises*.
        target = Rotation2d.fromDegrees(Math.min(Math.max(ArmConstants.MIN_ARM_DEGREES, target.getDegrees()), ArmConstants.MAX_ARM_DEGREES));

        //Invoke arm setting logic from ArmToPositionCommand
        super.execute();
    }
}

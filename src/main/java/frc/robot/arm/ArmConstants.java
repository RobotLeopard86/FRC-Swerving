package frc.robot.arm;

import frc.robot.Utils.PID;

public class ArmConstants {
    private ArmConstants() {}

    public static final double MAX_ARM_DEGREES = 50.0;
    public static final double MIN_ARM_DEGREES = -120.0;
    public static final double ARM_TOLERANCE_DEGREES = 2.0;
    public static final double ARM_START_DEGREES = -38.0;
    public static final double ARM_STOW_DEGREES = -83.0;
    public static final double ARM_AMP_SHOOT_DEGREES = -24;
    public static final double ARM_SPEAKER_SHOOT_DEGREES = -105.822;
    public static final double ARM_INTAKE_DEGREES = -110.5;
    public static final double ARM_MOVE_DEGREES = 1.0;
    public static final int XBOX_ID = 0;

    public static final PID ARM_PID = new PID(50.0, 0, 0);
    public static final double ARM_REDUCTION = (5.0 / 1.0) * (4.0 / 1.0) * (4.0 / 1.0) * (4.0 / 1.0);
}

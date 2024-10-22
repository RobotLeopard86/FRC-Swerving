package frc.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Utils.PID;

// Aceius: This could just be a nested class of RobotConstants.java
// Then it could also be static and final
public class DriveConstants {
    private DriveConstants() {}

    public record SwerveModuleConfig(int driveDevID, int rotateDevID, int cancoderID, Translation2d dfc, Rotation2d encoderOffset) {}

    public static final double DRIVE_REDUCTION = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);
    public static final double ROTATE_REDUCTION = (12.8 / 1.0);

    public static final int PIGEON2_ID = 40;

    public static final PID DRIVE_PID = new PID(0.000006, 0, 0);
    public static final PID ROTATE_PID = new PID(10, 0, 0.0002);

    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;
    public static final double BOT_WIDTH = 0.885;

    public static final SwerveModuleConfig FRONT_LEFT = new SwerveModuleConfig(2, 3, 3, new Translation2d(-1, 1), Rotation2d.fromRotations(0.881591796875 - 0.25));
    public static final SwerveModuleConfig FRONT_RIGHT = new SwerveModuleConfig(14, 17, 4, new Translation2d(1, 1), Rotation2d.fromRotations(-0.77758789062));
    public static final SwerveModuleConfig BACK_LEFT = new SwerveModuleConfig(8, 9, 2, new Translation2d(-1, -1), Rotation2d.fromRotations(-0.641357421875));
    public static final SwerveModuleConfig BACK_RIGHT = new SwerveModuleConfig(10, 11, 1, new Translation2d(1, -1), Rotation2d.fromRotations(-0.046142578125 + 0.5));
}

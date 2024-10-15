package frc.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    private DriveConstants() {}

    public record SwerveModuleConfig(int driveDevID, int rotateDevID, int cancoderID, Translation2d dfc, Rotation2d encoderOffset) {}
    public record PID(double p, double i, double d) {}

    public static final double driveGR = 8.14;
    public static final double rotateGR = 12.8;

    public static final int pigeon2ID = 40;

    public static final PID drivePid = new PID(0.000006, 0, 0);
    public static final PID rotatePid = new PID(10, 0, 0.0002);

    public static final double wheelCircumference = Units.inchesToMeters(4) * Math.PI;
    public static final double botWidth = 0.885;

    public static final SwerveModuleConfig frontLeft = new SwerveModuleConfig(2, 3, 3, new Translation2d(-1, 1), Rotation2d.fromRotations(0.881591796875 - 0.25));
    public static final SwerveModuleConfig frontRight = new SwerveModuleConfig(14, 17, 4, new Translation2d(1, 1), Rotation2d.fromRotations(-0.77758789062));
    public static final SwerveModuleConfig backLeft = new SwerveModuleConfig(8, 9, 2, new Translation2d(-1, -1), Rotation2d.fromRotations(-0.641357421875));
    public static final SwerveModuleConfig backRight = new SwerveModuleConfig(10, 11, 1, new Translation2d(1, -1), Rotation2d.fromRotations(-0.046142578125 + 0.5));
}

package frc.robot.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    private DriveConstants() {}

    public record SwerveModuleConfig(int driveDevID, int rotateDevID, int cancoderID, Translation2d dfc) {}
    public record PID(double p, double i, double d) {}

    public static final double driveGR = 8.14;
    public static final double rotateGR = 1;

    public static final PID drivePid = new PID(1, 0, 7.2);
    public static final PID rotatePid = new PID(3.716, 6, 9.376);

    public static final double wheelCircumference = Units.inchesToMeters(4) * Math.PI;
    public static final double botWidth = 0.885;

    public static final SwerveModuleConfig frontLeft = new SwerveModuleConfig(2, 3, 3, new Translation2d(-1, 1));
    public static final SwerveModuleConfig frontRight = new SwerveModuleConfig(14, 17, 4, new Translation2d(1, 1));
    public static final SwerveModuleConfig backLeft = new SwerveModuleConfig(8, 9, 2, new Translation2d(-1, -1));
    public static final SwerveModuleConfig backRight = new SwerveModuleConfig(10, 11, 1, new Translation2d(1, -1));
}

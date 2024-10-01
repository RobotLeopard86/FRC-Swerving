package frc.robot.drive;

public class DriveConstants {
    private DriveConstants() {}

    public record SwerveModuleConfig(int driveDevID, int rotateDevID, int cancoderID) {}
    public record PID(double p, double i, double d) {}

    public static final double driveGR = 8.14;
    public static final double rotateGR = 1;

    public static final PID drivePid = new PID(1, 0, 7.2);
    public static final PID rotatePid = new PID(3.716, 6, 9.376);

    public static final SwerveModuleConfig frontLeft = new SwerveModuleConfig(2, 3, 3);
    public static final SwerveModuleConfig frontRight = new SwerveModuleConfig(14, 17, 4);
    public static final SwerveModuleConfig backLeft = new SwerveModuleConfig(8, 9, 2);
    public static final SwerveModuleConfig backRight = new SwerveModuleConfig(10, 11, 1);
}

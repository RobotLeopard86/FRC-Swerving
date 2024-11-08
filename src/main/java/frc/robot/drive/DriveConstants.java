package frc.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Utils.PID;

public class DriveConstants {
        private DriveConstants() {
        }

        public record SwerveModuleConfig(int driveMotorID, int rotateMotorID, int absoluteEncoderID,
                        Translation2d distanceFromCenter, Rotation2d encoderOffset) {
        }

        public static final double DRIVE_REDUCTION = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);
        public static final double ROTATE_REDUCTION = (12.8 / 1.0);

        public static final int GYRO_ID = 40;

        public static final PID DRIVE_PID = new PID(0.000006, 0, 0);
        public static final PID ROTATE_PID = new PID(10, 0, 0.0002);

        public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2 * Math.PI;
        public static final double BOT_WIDTH = 0.885;

        public enum GyroType {
                NavX, Pigeon2
        }

        public static final double CONTROLLER_DEADBAND = 0.1;

        public static final GyroType GYRO_TYPE = GyroType.NavX;

        public static final SwerveModuleConfig FRONT_LEFT_SWERVE_CONFIG = new SwerveModuleConfig(2, 3, 3,
                        new Translation2d(BOT_WIDTH / 2, BOT_WIDTH / 2), Rotation2d.fromRotations(0.631591796875));
        public static final SwerveModuleConfig FRONT_RIGHT_SWERVE_CONFIG = new SwerveModuleConfig(14, 17, 4,
                        new Translation2d(BOT_WIDTH / 2, -BOT_WIDTH / 2), Rotation2d.fromRotations(-0.77758789062));
        public static final SwerveModuleConfig BACK_LEFT_SWERVE_CONFIG = new SwerveModuleConfig(8, 9, 2,
                        new Translation2d(-BOT_WIDTH / 2, BOT_WIDTH / 2), Rotation2d.fromRotations(-0.641357421875));
        public static final SwerveModuleConfig BACK_RIGHT_SWERVE_CONFIG = new SwerveModuleConfig(10, 11, 1,
                        new Translation2d(-BOT_WIDTH / 2, -BOT_WIDTH / 2), Rotation2d.fromRotations(0.453857421875));
}

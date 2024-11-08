package frc.robot.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private SwerveModule frontLeft, frontRight, backLeft, backRight;

    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveDriveKinematics kinematics;
    private GyroIO gyro;
    private Supplier<Double> gyroYawSupplier;

    private Pose2d pose;

    public SwerveDrive(Pose2d initialPose) {
        frontLeft = new SwerveModule(DriveConstants.FRONT_LEFT_SWERVE_CONFIG, "Front Left");
        frontRight = new SwerveModule(DriveConstants.FRONT_RIGHT_SWERVE_CONFIG, "Front Right");
        backLeft = new SwerveModule(DriveConstants.BACK_LEFT_SWERVE_CONFIG, "Back Left");
        backRight = new SwerveModule(DriveConstants.BACK_RIGHT_SWERVE_CONFIG, "Back Right");
        pose = initialPose;
        switch (DriveConstants.GYRO_TYPE) {
            case NavX:
                gyro = new NavXGyro();
                break;
            case Pigeon2:
                gyro = new PigeonGyro();
                break;
        }
        gyroYawSupplier = gyro.getYawSupplier();
        kinematics = new SwerveDriveKinematics(frontLeft.getDistanceFromRobotCenter(),
                frontRight.getDistanceFromRobotCenter(), backLeft.getDistanceFromRobotCenter(),
                backRight.getDistanceFromRobotCenter());
        SwerveModulePosition[] positions = { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.fromRotations(gyroYawSupplier.get()),
                positions, pose);
    }

    @Override
    public void periodic() {
        super.periodic();
        SwerveModulePosition[] positions = { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
        pose = poseEstimator.update(Rotation2d.fromRotations(gyroYawSupplier.get()), positions);
    }

    public Pose2d getPose() {
        return pose;
    }

    public void resetPose(Pose2d newPose) {
        pose = newPose;
        SwerveModulePosition[] positions = { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
        poseEstimator.resetPosition(Rotation2d.fromRotations(gyroYawSupplier.get()), positions, pose);
    }

    public void zeroGyro() {
        gyro.reset();
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> standardDeviations) {
        poseEstimator.setVisionMeasurementStdDevs(standardDeviations);
        addVisionMeasurement(visionPose, timestamp);
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }

    public ChassisSpeeds getCurrentRobotSpeeds() {
        return kinematics.toChassisSpeeds(frontLeft.getCurrentState(), frontRight.getCurrentState(),
                backLeft.getCurrentState(), backRight.getCurrentState());
    }

    public ChassisSpeeds getTargetRobotSpeeds() {
        return kinematics.toChassisSpeeds(frontLeft.getTargetState(), frontRight.getTargetState(),
                backLeft.getTargetState(), backRight.getTargetState());
    }

    public void setRobotSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        frontLeft.setTargetState(states[0]);
        frontRight.setTargetState(states[1]);
        backLeft.setTargetState(states[2]);
        backRight.setTargetState(states[3]);
    }

    public SwerveDriveWheelStates getCurrentWheelSpeeds() {
        return kinematics.toWheelSpeeds(getCurrentRobotSpeeds());
    }

    public SwerveDriveWheelStates getTargetWheelSpeeds() {
        return kinematics.toWheelSpeeds(getTargetRobotSpeeds());
    }

    public SwerveDriveWheelPositions getWheelPositions() {
        SwerveModulePosition[] smp = { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
        return new SwerveDriveWheelPositions(smp);
    }

    public void stop() {
        setRobotSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    public double getMaxLinearSpeedMetersPerSecond() {
        return 3.81;
    }

    public double getMaxAngularSpeedRadsPerSecond() {
        return getMaxLinearSpeedMetersPerSecond() / DriveConstants.BOT_WIDTH;
    }
}

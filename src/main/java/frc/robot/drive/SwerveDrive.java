package frc.robot.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private SwerveModule fl, fr, bl, br;

    private SwerveDrivePoseEstimator pe;
    private SwerveDriveKinematics kinematics;
    private AHRS ahrs;

    private Pose2d pose;

    public SwerveDrive(Pose2d initialPose) {
        fl = new SwerveModule(DriveConstants.frontLeft);
        fr = new SwerveModule(DriveConstants.frontRight);
        bl = new SwerveModule(DriveConstants.backLeft);
        br = new SwerveModule(DriveConstants.backRight);
        pose = initialPose;
        ahrs = new AHRS(Port.kMXP);
        kinematics = new SwerveDriveKinematics(fl.getDistanceFromCenter(), fr.getDistanceFromCenter(), bl.getDistanceFromCenter(), br.getDistanceFromCenter());
        SwerveModulePosition[] positions = {fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()};
        pe = new SwerveDrivePoseEstimator(kinematics, ahrs.getRotation2d(), positions, pose);
    }

    @Override
    public void periodic() {
        super.periodic();
        SwerveModulePosition[] positions = {fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()};
        pose = pe.update(ahrs.getRotation2d(), positions);
    }

    Pose2d getPose() {
        return pose;
    }

    void resetPose(Pose2d newPose) {
        pose = newPose;
        SwerveModulePosition[] positions = {fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()};
        pe.resetPosition(ahrs.getRotation2d(), positions, pose);
    }

    void zeroGyro() {
        ahrs.reset();
    }
    
    void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> standardDeviations) {
        pe.setVisionMeasurementStdDevs(standardDeviations);
        addVisionMeasurement(visionPose, timestamp);
    }

    void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        pe.addVisionMeasurement(visionPose, timestamp);
    }

    ChassisSpeeds getRobotSpeeds() {
        return kinematics.toChassisSpeeds(fl.getCurrentState(), fr.getCurrentState(), bl.getCurrentState(), br.getCurrentState());
    }
}

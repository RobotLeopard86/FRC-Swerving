package frc.robot.drive;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

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
    private SwerveModule fl, fr, bl, br;

    private SwerveDrivePoseEstimator pe;
    private SwerveDriveKinematics kinematics;
    private Pigeon2 pigeon;
    private Supplier<Double> pigeonYawSupplier;

    private Pose2d pose;

    public SwerveDrive(Pose2d initialPose) {
        fl = new SwerveModule(DriveConstants.FRONT_LEFT);
        fr = new SwerveModule(DriveConstants.FRONT_RIGHT);
        bl = new SwerveModule(DriveConstants.BACK_LEFT);
        br = new SwerveModule(DriveConstants.BACK_RIGHT);
        pose = initialPose;
        pigeon = new Pigeon2(DriveConstants.PIGEON2_ID);
        pigeonYawSupplier = pigeon.getYaw().asSupplier();
        kinematics = new SwerveDriveKinematics(fl.getDistanceFromCenter(), fr.getDistanceFromCenter(), bl.getDistanceFromCenter(), br.getDistanceFromCenter());
        SwerveModulePosition[] positions = {fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()};
        pe = new SwerveDrivePoseEstimator(kinematics, Rotation2d.fromRotations(pigeonYawSupplier.get()), positions, pose);
    }

    @Override
    public void periodic() {
        super.periodic();
        SwerveModulePosition[] positions = {fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()};
        pose = pe.update(Rotation2d.fromRotations(pigeonYawSupplier.get()), positions);
    }

    public Pose2d getPose() {
        return pose;
    }

    public void resetPose(Pose2d newPose) {
        pose = newPose;
        SwerveModulePosition[] positions = {fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()};
        pe.resetPosition(Rotation2d.fromRotations(pigeonYawSupplier.get()), positions, pose);
    }

    public void zeroGyro() {
        pigeon.reset();
    }
    
    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> standardDeviations) {
        pe.setVisionMeasurementStdDevs(standardDeviations);
        addVisionMeasurement(visionPose, timestamp);
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        pe.addVisionMeasurement(visionPose, timestamp);
    }

    public ChassisSpeeds getCurrentRobotSpeeds() {
        return kinematics.toChassisSpeeds(fl.getCurrentState(), fr.getCurrentState(), bl.getCurrentState(), br.getCurrentState());
    }

    public ChassisSpeeds getTargetRobotSpeeds() {
        return kinematics.toChassisSpeeds(fl.getTargetState(), fr.getTargetState(), bl.getTargetState(), br.getTargetState());
    }

    public void setRobotSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        fl.setTargetState(states[0]);
        fr.setTargetState(states[1]);
        bl.setTargetState(states[2]);
        br.setTargetState(states[3]);
    }

    public SwerveDriveWheelStates getCurrentWheelSpeeds() {
        return kinematics.toWheelSpeeds(getCurrentRobotSpeeds());
    }

    public SwerveDriveWheelStates getTargetWheelSpeeds() {
        return kinematics.toWheelSpeeds(getTargetRobotSpeeds());
    }

    public SwerveDriveWheelPositions getWheelPositions() {
        SwerveModulePosition[] smp = {fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()};
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

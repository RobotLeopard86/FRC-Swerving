package frc.robot.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.drive.DriveConstants.SwerveModuleConfig;

public class SwerveModule {
    private CANSparkMax rotateMotor, driveMotor;
    private CANcoder absoluteEncoder;

    private SwerveModuleState state;

    private Translation2d distanceFromRobotCenter;

    public SwerveModule(SwerveModuleConfig cfg) {
        driveMotor = new CANSparkMax(cfg.driveMotorID(), MotorType.kBrushless);
        rotateMotor = new CANSparkMax(cfg.rotateMotorID(), MotorType.kBrushless);

        state = new SwerveModuleState();

        absoluteEncoder = new CANcoder(cfg.absoluteEncoderID());
        absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withMagnetOffset(cfg.encoderOffset().getRotations())));

        driveMotor.getEncoder().setPositionConversionFactor(1 / DriveConstants.DRIVE_REDUCTION);
        driveMotor.getEncoder().setVelocityConversionFactor(1 / DriveConstants.DRIVE_REDUCTION);
        driveMotor.getPIDController().setP(DriveConstants.DRIVE_PID.p(), 0);
        driveMotor.getPIDController().setI(DriveConstants.DRIVE_PID.i(), 0);
        driveMotor.getPIDController().setD(DriveConstants.DRIVE_PID.d(), 0);

        rotateMotor.getEncoder().setPositionConversionFactor(1 / DriveConstants.ROTATE_REDUCTION);
        rotateMotor.getEncoder().setVelocityConversionFactor(1 / DriveConstants.ROTATE_REDUCTION);
        rotateMotor.getEncoder().setPosition(absoluteEncoder.getPosition().getValueAsDouble());
        rotateMotor.getPIDController().setPositionPIDWrappingEnabled(true);
        rotateMotor.getPIDController().setPositionPIDWrappingMaxInput(1);
        rotateMotor.getPIDController().setPositionPIDWrappingMinInput(0);
        rotateMotor.getPIDController().setP(DriveConstants.ROTATE_PID.p(), 0);
        rotateMotor.getPIDController().setI(DriveConstants.ROTATE_PID.i(), 0);
        rotateMotor.getPIDController().setD(DriveConstants.ROTATE_PID.d(), 0);

        distanceFromRobotCenter = cfg.distanceFromCenter();

        driveMotor.restoreFactoryDefaults();
        driveMotor.setIdleMode(IdleMode.kBrake);

        rotateMotor.restoreFactoryDefaults();
        rotateMotor.setIdleMode(IdleMode.kBrake);
    }

    public Translation2d getDistanceFromRobotCenter() {
        return distanceFromRobotCenter;
    }

    public void setTargetState(SwerveModuleState newState) {
        state = newState;
        rotateMotor.getPIDController().setReference(state.angle.getRotations(), ControlType.kPosition, 0);
        driveMotor.getPIDController().setReference(state.speedMetersPerSecond / DriveConstants.WHEEL_CIRCUMFERENCE, ControlType.kVelocity, 0);
        SmartDashboard.putNumber("rotate", state.angle.getRotations());
        SmartDashboard.putNumber("drive", state.speedMetersPerSecond);
    }

    public SwerveModuleState getTargetState() {
        return state;
    }

    public SwerveModuleState getCurrentState() {
       return new SwerveModuleState(driveMotor.getEncoder().getVelocity(), new Rotation2d(rotateMotor.getEncoder().getPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(DriveConstants.WHEEL_CIRCUMFERENCE * driveMotor.getEncoder().getPosition(), new Rotation2d(rotateMotor.getEncoder().getPosition()));
    }

    public void stop() {
        driveMotor.stopMotor();
        rotateMotor.stopMotor();
    }

}

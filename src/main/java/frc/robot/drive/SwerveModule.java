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
    private CANSparkMax rotate, drive;
    private CANcoder cancoder;

    private SwerveModuleState state;

    private Translation2d distFromCtr;

    public SwerveModule(int rotateDev, int driveDev, int cancoderID, Translation2d dfc, Rotation2d encoderOffset) {
        drive = new CANSparkMax(driveDev, MotorType.kBrushless);
        rotate = new CANSparkMax(rotateDev, MotorType.kBrushless);
        state = new SwerveModuleState();
        drive.getEncoder().setPositionConversionFactor(1 / DriveConstants.DRIVE_REDUCTION);
        drive.getEncoder().setVelocityConversionFactor(1 / DriveConstants.DRIVE_REDUCTION);
        drive.getPIDController().setP(DriveConstants.DRIVE_PID.p());
        drive.getPIDController().setI(DriveConstants.DRIVE_PID.i());
        drive.getPIDController().setD(DriveConstants.DRIVE_PID.d());
        rotate.getEncoder().setPositionConversionFactor(1 / DriveConstants.ROTATE_REDUCTION);
        rotate.getEncoder().setVelocityConversionFactor(1 / DriveConstants.ROTATE_REDUCTION);
        cancoder = new CANcoder(cancoderID);
        cancoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive).withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1).withMagnetOffset(encoderOffset.getRotations())));
        rotate.getEncoder().setPosition(cancoder.getPosition().getValueAsDouble());
        rotate.getPIDController().setPositionPIDWrappingEnabled(true);
        rotate.getPIDController().setPositionPIDWrappingMaxInput(1);
        rotate.getPIDController().setPositionPIDWrappingMinInput(0);
        rotate.getPIDController().setP(DriveConstants.ROTATE_PID.p());
        rotate.getPIDController().setI(DriveConstants.ROTATE_PID.i());
        rotate.getPIDController().setD(DriveConstants.ROTATE_PID.d());
        distFromCtr = dfc;
        drive.restoreFactoryDefaults();
        drive.setIdleMode(IdleMode.kBrake);
        rotate.restoreFactoryDefaults();
        rotate.setIdleMode(IdleMode.kBrake);
    }

    public SwerveModule(SwerveModuleConfig cfg) {
        this(cfg.rotateDevID(), cfg.driveDevID(), cfg.cancoderID(), cfg.dfc(), cfg.encoderOffset());
    }

    Translation2d getDistanceFromCenter() {
        return distFromCtr;
    }

    void setTargetState(SwerveModuleState newState) {
        state = newState;
        rotate.getPIDController().setReference(state.angle.getRotations(), ControlType.kPosition, 0);
        drive.getPIDController().setReference(state.speedMetersPerSecond, ControlType.kVelocity, 0);
        SmartDashboard.putNumber("rotate", state.angle.getRotations());
        SmartDashboard.putNumber("drive", state.speedMetersPerSecond);
    }

    SwerveModuleState getTargetState() {
        return state;
    }

    SwerveModuleState getCurrentState() {
       return new SwerveModuleState(drive.getEncoder().getVelocity(), new Rotation2d(rotate.getEncoder().getPosition()));
    }

    SwerveModulePosition getPosition() {
        return new SwerveModulePosition(DriveConstants.WHEEL_CIRCUMFERENCE * drive.getEncoder().getPosition(), new Rotation2d(rotate.getEncoder().getPosition()));
    }

    void stop() {
        drive.stopMotor();
        rotate.stopMotor();
    }

}

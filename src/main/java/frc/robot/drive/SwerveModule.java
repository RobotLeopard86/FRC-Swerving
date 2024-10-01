package frc.robot.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.drive.DriveConstants.SwerveModuleConfig;

public class SwerveModule {
    private CANSparkMax rotate, drive;
    private CANcoder cancoder;

    private SwerveModuleState state;

    public SwerveModule(int rotateDev, int driveDev, int cancoderID) {
        drive = new CANSparkMax(driveDev, MotorType.kBrushless);
        rotate = new CANSparkMax(rotateDev, MotorType.kBrushless);
        state = new SwerveModuleState();
        drive.getEncoder().setPositionConversionFactor(1 / DriveConstants.driveGR);
        drive.getPIDController().setP(DriveConstants.drivePid.p());
        drive.getPIDController().setI(DriveConstants.drivePid.i());
        drive.getPIDController().setD(DriveConstants.drivePid.d());
        rotate.getEncoder().setPositionConversionFactor(1 / DriveConstants.rotateGR);
        cancoder = new CANcoder(cancoderID);
        rotate.getEncoder().setPosition(cancoder.getPosition().getValueAsDouble());
        rotate.getPIDController().setPositionPIDWrappingEnabled(true);
        rotate.getPIDController().setPositionPIDWrappingMaxInput(1);
        rotate.getPIDController().setPositionPIDWrappingMinInput(0);
        rotate.getPIDController().setP(DriveConstants.rotatePid.p());
        rotate.getPIDController().setI(DriveConstants.rotatePid.i());
        rotate.getPIDController().setD(DriveConstants.rotatePid.d());
    }

    public SwerveModule(SwerveModuleConfig cfg) {
        this(cfg.rotateDevID(), cfg.driveDevID(), cfg.cancoderID());
    }

    CANSparkMax getRotateMotor() {
        return rotate;
    }

    CANSparkMax getDriveMotor() {
        return drive;
    }

    CANcoder getCancoder() {
        return cancoder;
    }

    void setState(SwerveModuleState newState) {
        state = newState;
        rotate.getPIDController().setReference(state.angle.getRotations(), ControlType.kPosition);
        drive.getPIDController().setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    }

    SwerveModuleState getState() {
        return state;
    }

}

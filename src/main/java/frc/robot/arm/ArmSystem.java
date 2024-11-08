package frc.robot.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSystem extends SubsystemBase {
    private CANSparkMax left, right;
    private CANcoder cancoder;

    private static final int leftID = 5, rightID = 19, cancoderID = 6;

    public ArmSystem() {
        left = new CANSparkMax(leftID, MotorType.kBrushless);
        left.restoreFactoryDefaults();
        left.setIdleMode(IdleMode.kBrake);
        left.getEncoder().setPositionConversionFactor(1 / ArmConstants.ARM_REDUCTION);
        left.getEncoder().setVelocityConversionFactor(1 / ArmConstants.ARM_REDUCTION);
        left.getPIDController().setP(ArmConstants.ARM_PID.p());
        left.getPIDController().setI(ArmConstants.ARM_PID.i());
        left.getPIDController().setD(ArmConstants.ARM_PID.d());
        right = new CANSparkMax(rightID, MotorType.kBrushless);
        right.restoreFactoryDefaults();
        right.setIdleMode(IdleMode.kBrake);
        right.getEncoder().setPositionConversionFactor(1 / ArmConstants.ARM_REDUCTION);
        right.getEncoder().setVelocityConversionFactor(1 / ArmConstants.ARM_REDUCTION);
        right.getPIDController().setP(ArmConstants.ARM_PID.p());
        right.getPIDController().setI(ArmConstants.ARM_PID.i());
        right.getPIDController().setD(ArmConstants.ARM_PID.d());
        cancoder = new CANcoder(cancoderID);
        right.follow(left, true);
        left.getEncoder().setPosition(Rotation2d.fromDegrees(ArmConstants.ARM_START_DEGREES).getRotations());
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(cancoder.getPosition().getValueAsDouble());
    }

    public void setCurrentAngle(Rotation2d newPosition) {
        // left.getPIDController().setReference(newPosition.getRotations(),
        // ControlType.kPosition);
        SmartDashboard.putString("ARM ANGLE TARGET DEGREES", String.valueOf(newPosition.getDegrees()));
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putString("ARM ANGLE CURRENT DEGREES", String
                .valueOf(Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble()).getDegrees()));
    }
}

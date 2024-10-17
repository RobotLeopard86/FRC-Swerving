package frc.robot.arm;

import com.ctre.phoenix6.hardware.CANcoder;
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
        right = new CANSparkMax(rightID, MotorType.kBrushless);
        cancoder = new CANcoder(cancoderID);
        right.follow(left, true);
        left.getEncoder().setPosition(Rotation2d.fromDegrees(ArmConstants.armStartDegrees).getRotations());
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(cancoder.getPosition().getValueAsDouble());
    }

    public void setCurrentAngle(Rotation2d newPosition) {
        cancoder.setPosition(newPosition.getRotations());
        SmartDashboard.putString("ARM POSITION DEGREES", String.valueOf(newPosition.getDegrees()));
    }
}

package frc.robot.drive;

import java.util.function.Supplier;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.SerialPort;

public class PigeonGyro implements GyroIO {
    private Pigeon2 pigeon;

    @Override
    public Supplier<Double> getYawSupplier() {
        return pigeon.getYaw().asSupplier();
    }

    @Override
    public void reset() {
        pigeon.reset();
    }

    public PigeonGyro() {
        pigeon = new Pigeon2(DriveConstants.GYRO_ID);
    }

}

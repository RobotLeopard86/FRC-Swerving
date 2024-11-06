package frc.robot.drive;

import java.util.function.Supplier;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;

public class NavXGyro implements GyroIO {
    private AHRS ahrs;

    @Override
    public Supplier<Double> getYawSupplier() {
        return () -> ahrs.getAngle();
    }

    @Override
    public void reset() {
        a.reset();
    }

    public NavXGyro() {
        ahrs = new AHRS(SerialPort.Port.kMXP);
    }

}

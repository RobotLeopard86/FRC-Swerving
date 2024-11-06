package frc.robot.drive;

import java.util.function.Supplier;

public interface GyroIO {
    Supplier<Double> getYawSupplier();

    void reset();
}

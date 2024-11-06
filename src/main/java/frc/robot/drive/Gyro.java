package frc.robot.drive;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

public class Gyro<T> {
    private T hardwareGyro;
    private Function<T, Supplier<Double>> providerFunction;
    private Consumer<T> resetFunction;

    public Gyro(T hardware, Function<T, Supplier<Double>> providerFunc, Consumer<T> resetFunc) {
        hardwareGyro = hardware;
        providerFunction = providerFunc;
        resetFunction = resetFunc;
    }

    public Supplier<Double> getYawSupplier() {
        return providerFunction.apply(hardwareGyro);
    }

    public void reset() {
        resetFunction.accept(hardwareGyro);
    }
}

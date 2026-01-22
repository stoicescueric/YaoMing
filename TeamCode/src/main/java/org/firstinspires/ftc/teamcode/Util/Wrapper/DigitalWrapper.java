package org.firstinspires.ftc.teamcode.Util.Wrapper;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.Controllers.RingBuffer;
import org.firstinspires.ftc.teamcode.Util.Math.Debouncer;


public class DigitalWrapper {
    private static final int BUFFER_SIZE = 5;

    private Debouncer debouncer;
    private DigitalChannel device;
    private RingBuffer<Boolean> ringBuffer;

    public DigitalWrapper(HardwareMap hardwareMap, String name) {
        device = hardwareMap.get(DigitalChannel.class, name);
        device.setMode(DigitalChannel.Mode.INPUT);

        debouncer = new Debouncer(0.2, Debouncer.DebounceType.kBoth);
        ringBuffer = new RingBuffer<>(BUFFER_SIZE, false);
    }

    public boolean getValue() {

        boolean debouncedValue = debouncer.calculate(getRaw());

        ringBuffer.getValue(debouncedValue);

        boolean allSame = ringBuffer.getList().stream().allMatch(val -> val == debouncedValue);

        return allSame == debouncedValue;
    }
    public boolean getRaw() {
        return !device.getState();
    }
}
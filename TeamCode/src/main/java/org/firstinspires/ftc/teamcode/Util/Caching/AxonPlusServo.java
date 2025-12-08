package org.firstinspires.ftc.teamcode.Util.Caching;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;

import java.util.ArrayList;
import java.util.List;



public class AxonPlusServo implements CRServo {
    private CRServo delegateServo;
    public AnalogInput delegateEncoder;
    MultipleTelemetry telem;

    private double cachedPower = 0;
    private double lastNonNullPower = 0;
    private double cachedPosition = 0;
    private double absolutePosition = 0;
    private int direction = 1;
    public double rotations = 0;

    public double diff = 0;

    public double speedUp = 0.8;
    public double speedDown = -0.8;
    public double iterationLimitUp = 360; //45
    public double iterationLimitDown = 360; //20

    private  List<String> changes = new ArrayList<String>();
    private  List<String> positions = new ArrayList<String>();

    //    public AxonPlusServo(CRServo servo, AnalogInput encoder, MultipleTelemetry _telem) {
    public AxonPlusServo(CRServo servo, AnalogInput encoder) {
        delegateServo = servo;
        delegateEncoder = encoder;
        cachedPosition = delegateEncoder.getVoltage() / 3.3 * 360                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   ;
        setAbsolutePosition(getRelativePosition());
//        telem = _telem;
    }

    public double getVoltage() {
        return delegateEncoder.getVoltage();
    }

    public double getRelativePosition() {
        return delegateEncoder.getVoltage() / 3.3 * 360;
    }

    public double getCachedPosition() {
        return cachedPosition;
    }

    public double getAbsolutePosition() {
        return absolutePosition;
    }

    public void setAbsolutePosition(double _absolutePosition) {
        absolutePosition = _absolutePosition;
    }

    public void updatePower(double target,double position) {
        double error = target-position;
        if(target - (position + 100)<=100 || target - (position - 100)<=100 || target - position<=100) {
            setPower(0);
        }
        if(error > 0) {
            if(cachedPower!=speedUp) {
                setPower(speedUp);
            }
            cachedPower = speedUp;
        }else {
            if(cachedPower!=speedDown) {
                setPower(speedDown);
            }
            cachedPower = speedDown;
        }
    }
    public void update2() {
        double delta = getRelativePosition() - getCachedPosition();

        setAbsolutePosition(absolutePosition + delta);
        cachedPosition = getRelativePosition();


    }

    public void ericUpdate() {
        double currentRelative = getRelativePosition();
        double power = getPower();
        if(Math.abs(currentRelative - cachedPosition) >= 2) {
            if (power < 0) {
                //relative creste
                double dif;
                if (currentRelative >= cachedPosition) {
                    dif = (currentRelative - cachedPosition);
                } else {
                    //inseamna ca a fct full circle
                    dif = (currentRelative + (360 - cachedPosition));
                }
                absolutePosition+=dif;
            } else {
                //relative scade
                double dif;
                if (currentRelative <= cachedPosition) {
                    dif = cachedPosition - currentRelative;
                } else {
                    dif = cachedPosition - (360 - currentRelative);
                }
                absolutePosition-=dif;
            }
            cachedPosition = currentRelative;
        }
    }
    public void update() {
        double newPosition = getRelativePosition();
        if (newPosition > cachedPosition + 3.0 ||
                newPosition < cachedPosition - 3.0) {
            changes.add("pos" + String.valueOf(newPosition));
        }

        double val1 = 360 - cachedPosition + newPosition;
        double val2 = newPosition - cachedPosition;
        double val3 = 360 - newPosition + cachedPosition;
        double val4 = cachedPosition - newPosition;

        if (0 <= val1 &&
                (val1 < val2 || val2 < 0) &&
                (val1 < val3 || val3 < 0) &&
                (val1 < val4 || val4 < 0)) {
            rotations++;
            absolutePosition = absolutePosition +
                    val1;
            diff = val1;
            cachedPosition = newPosition;
        } else if (0 <= val2 &&
                (val2 < val1 || val1 < 0) &&
                (val2 < val3 || val3 < 0) &&
                (val2 < val4 || val4 < 0)) {
            absolutePosition = absolutePosition +
                    val2;
            diff = val2;
            cachedPosition = newPosition;
        }
        if (0 <= val3 &&
                (val3 < val1 || val1 < 0) &&
                (val3 < val2 || val2 < 0) &&
                (val3 < val4 || val4 < 0)) {
            rotations--;
            absolutePosition = absolutePosition -
                    val3;
            diff = val3;
            cachedPosition = newPosition;
        } else if (0 <= val4 &&
                (val4 < val1 || val1 < 0) &&
                (val4 < val2 || val2 < 0) &&
                (val4 < val3 || val3 < 0)) {
            absolutePosition = absolutePosition -
                    val4;
            diff = val4;
            cachedPosition = newPosition;
        }

        if(changes.size()>=10){
            changes.remove(0);
        }
        if(positions.size()>=10){
            positions.remove(0);
        }

//        telem.addData("diff ", diff);
//        telem.addLine("Changes:");
//        for(String change:changes){
//            telem.addLine(change);
//        }
//        telem.addLine("cachedpos:");
//        for(String pos:positions){
//            telem.addLine(pos);
//        }
    }

    @Override
    public ServoController getController() {
        return delegateServo.getController();
    }

    @Override
    public int getPortNumber() {
        return delegateServo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        delegateServo.setDirection(direction);
    }

    public void setDirection(int dir) {
        direction = dir;
        delegateServo.setDirection(Direction.REVERSE);
    }

    public void changeDiretion() {
        if(delegateServo.getDirection() == Direction.FORWARD) {
            delegateServo.setDirection(Direction.REVERSE);
        }else {
            delegateServo.setDirection(Direction.FORWARD);
        }
    }

    @Override
    public void setPower(double power) {
        if(power!=cachedPower) {
            delegateServo.setPower(power);
        }
        cachedPower = power;

    }

    @Override
    public double getPower() {
        return delegateServo.getPower();
    }

    public double getCachedPower() {
        return cachedPower;
    }

    public double getLastNonNullPower() {
        return lastNonNullPower;
    }

    @Override
    public Direction getDirection() {
        return delegateServo.getDirection();
    }

    @Override
    public Manufacturer getManufacturer() {
        return delegateServo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return delegateServo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return delegateServo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return delegateServo.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        delegateServo.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        delegateServo.close();
    }
}
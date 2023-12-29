package org.firstinspires.ftc.teamcode.facade.drive;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class absoluteAnalogEncoder {

    private final AnalogInput encoder;
    private double offset;
    private boolean inverted;

    public absoluteAnalogEncoder(AnalogInput enc, double off, boolean invert){
        encoder = enc;
        offset =off;
        inverted = invert;
    }

    public double getCurrentPosition() {
        double pos =  (!inverted ? 1 - getVoltage() / 3.3 : getVoltage() / 3.3) * 360 - offset;
        if ( pos < 0 )
            pos+=360.0d;
        if ( pos > 360 )
            pos %= 360;
        return pos;
    }
    public boolean getDirection() {
        return inverted;
    }
    public AnalogInput getEncoder() {
        return encoder;
    }
    public double getVoltage(){
        return encoder.getVoltage();
    }

}
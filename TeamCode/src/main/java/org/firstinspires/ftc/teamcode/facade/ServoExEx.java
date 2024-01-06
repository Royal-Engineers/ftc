package org.firstinspires.ftc.teamcode.facade;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ServoExEx {
    ServoEx m_Servo;

    public ServoExEx(HardwareMap map, String name, double pos1, double pos2, double InitPos)
    {
        m_Servo = new SimpleServo(map, name, pos1, pos2);
        m_Servo.setPosition(InitPos);
    }

    public void setPosition(double pos)
    {
        double CurrentPos = getPosition();
        m_Servo.rotateBy(pos - CurrentPos);

    }

    public double getPosition()
    {
        return m_Servo.getPosition();
    }

    public double getAngle()
    {
        return m_Servo.getAngle();
    }

    public boolean getInverted()
    {
        return m_Servo.getInverted();
    }

    public void setInverted(boolean inv)
    {
        m_Servo.setInverted(inv);
    }

    public String toString()
    {
        return m_Servo.toString();
    }

}

package org.firstinspires.ftc.teamcode.facade.intake;

import com.arcrobotics.ftclib.hardware.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;

public class Bar {

    private RobotHardware m_Robot;
    private Telemetry m_Telemetry;
    private ServoEx m_MainServo, m_SecServo;

    private double m_MainPositon = 0.0d, m_InvPosition = 0.0d;

    public Bar(RobotHardware robot, ServoEx MainServo, ServoEx SecServo, double MainPosition, double InvPosition)
    {
        m_Robot = robot;
        m_MainServo = MainServo;
        m_SecServo = SecServo;
        m_Telemetry = robot.m_telemetry;

        m_MainPositon = MainPosition;
        m_InvPosition = InvPosition;
    }

    private double f_Position = 0.0d;
    public void SetPosition(double pos)
    {
        if (  m_MainPositon + pos < 0.0d || m_InvPosition + pos < 0.0d || m_MainPositon + pos > 1.0d || m_InvPosition + pos > 1.0d)
            return;
        f_Position = pos;

        m_MainServo.setPosition(m_MainPositon + pos);
        m_SecServo.setPosition(m_InvPosition + pos);
    }

    public double GetPosition()
    {
        return f_Position;
    }
    public void UpdateTelemetry()
    {
        m_Telemetry.addData("BOMBA SECY DEBUG INFO", "->START");
        m_Telemetry.addData("POSITION sexy scale", f_Position);
        m_Telemetry.addData("SERVO MAIN REAL POS->", m_MainServo.getPosition());
        m_Telemetry.addData("SERVO BOMBASTIC REAL POS", m_SecServo.getPosition());
        m_Telemetry.addData("BOMBA SECY DEBUG INFO", "->END\n");

    }



}

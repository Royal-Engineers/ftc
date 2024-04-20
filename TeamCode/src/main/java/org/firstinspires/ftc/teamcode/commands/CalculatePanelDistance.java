package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;

public class CalculatePanelDistance extends CommandBase {

    public static double DistanceIncrementToPanel = 0.0d;
    RobotHardware m_Robot;
    Telemetry m_Telemetry;

    DistanceSensor m_DistanceSensor;
    public CalculatePanelDistance(RobotHardware robothardware)
    {
        m_Robot = robothardware;
    }

    @Override
    public void initialize()
    {
     m_Telemetry = m_Robot.m_telemetry;
     DistanceIncrementToPanel = 0.0d;
     m_DistanceSensor = m_Robot.m_DistanceSensor;
    }

    @Override
    public void execute()
    {
        DistanceIncrementToPanel = m_DistanceSensor.getDistance(DistanceUnit.CM);
    }

    @Override
    public void finalize()
    {

    }

    @Override
    public boolean isFinished()
    {
        return true;
    }

}

package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;

public class DistanceSensorCommand extends CommandBase {

    public RobotHardware m_RobotHardware;
    public Telemetry m_Telemetry;
    public DistanceSensor m_DistanceSensor;

    public double DistanceNeeded = 90;
    public int Frames, FramesNOK = 1000;
    public static boolean ok = false;
    public final int NeededFrames = 10;

    public final int FramesForReset = 5;

    public DistanceSensorCommand(RobotHardware robothardware)
    {
        m_RobotHardware = robothardware;
    }

    @Override
    public void initialize()
    {
        m_DistanceSensor = m_RobotHardware.m_DistanceSensor;
        m_Telemetry = m_RobotHardware.m_telemetry;
    }

    @Override
    public void execute()
    {
        if ( DistanceNeeded > m_DistanceSensor.getDistance(DistanceUnit.MM) )
        {
            Frames++;
            FramesNOK = 0;
        }
        else {
            FramesNOK++;
            if ( FramesNOK > FramesForReset ) {
                Frames = 0;
                FramesNOK = 0;
            }
        }
        if ( Frames > NeededFrames )
            ok = true;
        else ok = false;

        m_Telemetry.addData("Distance", m_DistanceSensor.getDistance(DistanceUnit.MM));
        m_Telemetry.addData("DistanceSensorFrames", Frames);
    }

    @Override
    public void finalize()
    {

    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

}

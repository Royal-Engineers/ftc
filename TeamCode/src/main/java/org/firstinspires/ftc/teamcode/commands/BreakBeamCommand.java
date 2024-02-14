package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.executables.TeleOp;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;

import com.qualcomm.robotcore.hardware.TouchSensor;
public class BreakBeamCommand extends CommandBase {
    RobotHardware m_RobotHardware;

    Telemetry m_Telemetry;

    public int FramesNeeded = 150;

    public  int Frames = 0;
    TouchSensor m_BreakBeam;
    public BreakBeamCommand(RobotHardware robothardware)
    {
        m_RobotHardware = robothardware;
    }

    @Override
    public void initialize()
    {
        m_BreakBeam = m_RobotHardware.BreakBeam;
        m_Telemetry = m_RobotHardware.m_telemetry;
    }

    @Override
    public void execute()
    {
        Frames++;
        if (!m_BreakBeam.isPressed() )
           Frames = 0;
        else Frames++;

        if ( Frames == FramesNeeded && Math.abs(m_RobotHardware.motorIntake.getPower()) > 0.01 )
            m_RobotHardware.motorIntake.setPower(0);
        m_Telemetry.addData("BreakBeam frames", Frames);
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

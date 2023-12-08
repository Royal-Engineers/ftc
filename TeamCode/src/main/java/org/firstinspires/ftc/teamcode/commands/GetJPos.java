package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;

public class GetJPos extends CommandBase {

    public static boolean Enabled = false;

    public GetJPos(RobotHardware robot)
    {
        m_Robot = robot;
        m_Telemetry = m_Robot.m_telemetry;
        GetJPos.Enabled = true;
    }

    @Override
    public void initialize()
    {
        GetJPos.Enabled = true;
    }

    @Override
    public void execute()
    {

        m_Telemetry.addData("LeftX", m_Robot.m_gamepad1.left_stick_x);
        m_Telemetry.addData("LeftY", m_Robot.m_gamepad1.left_stick_y);
        m_Telemetry.addData("RightX", m_Robot.m_gamepad1.right_stick_x);
    }

    @Override
    public boolean isFinished()
    {
        return !GetJPos.Enabled;
    }

    private RobotHardware m_Robot;
    private Telemetry m_Telemetry;
}

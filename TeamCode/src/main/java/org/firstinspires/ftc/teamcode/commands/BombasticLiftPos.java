package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.facade.intake.Lift;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;

public class BombasticLiftPos extends CommandBase {
    private RobotHardware m_Robot;
    private Lift m_Lift;

    private Lift.e_LiftPosition m_StatePosition;
    public BombasticLiftPos(RobotHardware robot, Lift lift, Lift.e_LiftPosition StatePosition)
    {
        m_Robot = robot;
        m_Lift = lift;
        m_StatePosition = StatePosition;
    }

    @Override
    public void initialize()
    {
        m_Lift.SetStatePosition(m_StatePosition);
    }

    @Override
    public void execute()
    {
        m_Robot.m_telemetry.addData("Lift moving", "!!!");
    }

    @Override
    public boolean isFinished()
    {
        return m_Lift.IsWithinTolerance();
    }

}

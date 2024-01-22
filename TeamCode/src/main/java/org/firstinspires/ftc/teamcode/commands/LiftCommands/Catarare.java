package org.firstinspires.ftc.teamcode.commands.LiftCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.facade.RobotHardware;

public class Catarare extends CommandBase {
    double target = 0.0d;
    double toleranta = 30.0d;
    RobotHardware m_Robot;
    public Catarare(RobotHardware robot, double tget)
    {
        m_Robot = robot;
        target = tget;
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {
        if (  Math.abs(m_Robot.m_Lift.GetLiftPosition() - target) > toleranta)
        m_Robot.m_Lift.SetRawPower(-1.0d);
        else m_Robot.m_Lift.SetRawPower(0.0d);
    }

    @Override
    public boolean isFinished()
    {
        if ( Math.abs(m_Robot.m_Lift.GetLiftPosition() - target) < toleranta)
            return true;
        return false;
    }

    @Override
    public void finalize()
    {
        m_Robot.m_Lift.SetRawPower(0.0d);
    }

}

package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.drive.OdometryComponent;
import org.firstinspires.ftc.teamcode.facade.drive.DriveSubsystem;

public class GotoX extends CommandBase {

    Telemetry m_telemetry;
    double P = 0.01d, I = 0.005d, D = 0.0d;

    DriveSubsystem m_DriveSubsystem;
    PIDController pid;

    private double m_Target = 0.0d;

    RobotHardware m_Robot;
    public GotoX(double x, Telemetry telemetry, DriveSubsystem driveSubsystem, RobotHardware robot)
    {
        pid = new PIDController(P, I, D);
        pid.setSetPoint(x);
        m_telemetry = telemetry;
        m_DriveSubsystem = driveSubsystem;
        m_Target = x;
        m_Robot = robot;
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {
        double CurrentPos = OdometryComponent.X;
        m_DriveSubsystem.UpdateAuto(pid.calculate(CurrentPos), 0.0d, 0.0d);
        m_Robot.m_telemetry.addData("Target:", m_Target);

    }

    @Override
    public boolean isFinished()
    {

        if( Math.abs(OdometryComponent.X - m_Target) < 10.0d)
        {
            m_DriveSubsystem.UpdateAuto(0, 0, 0);
            return true;
        }
        return false;
    }
}

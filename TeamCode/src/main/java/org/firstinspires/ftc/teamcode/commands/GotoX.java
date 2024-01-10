package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.drive.OdometryComponent;
import org.firstinspires.ftc.teamcode.facade.drive.DriveSubsystem;

public class GotoX extends CommandBase {

    Telemetry m_telemetry;
    double P = 0.035d, I = 0.016d, D = 0.001d;

    DriveSubsystem m_DriveSubsystem;
    PIDController pid;

    private double m_Target = 0.0d;
    public double Power = 0.0d;

    public boolean active = true;
    RobotHardware m_Robot;
    public GotoX(double x, Telemetry telemetry, DriveSubsystem driveSubsystem, RobotHardware robot)
    {
        pid = new PIDController(P, I, D);
        pid.setSetPoint(x);
        m_telemetry = telemetry;
        m_DriveSubsystem = driveSubsystem;
        m_Target = x;
        m_Robot = robot;
        active = true;
    }

    @Override
    public void initialize()
    {

    }

    public void set(double pos)
    {
        m_Target = pos;
        pid.setSetPoint(pos);
        active = true;

    }
    @Override
    public void execute()
    {
        if ( !active ) {
            Power = 0.0d;
            return;
        }
        if( Math.abs(OdometryComponent.X - m_Target) < 5.0d) {
            active = false;
            Power = 0.0d;
            return;
        }
        double CurrentPos = OdometryComponent.X;
        Power = pid.calculate(CurrentPos);
        m_Robot.m_telemetry.addData("TargetX:", m_Target);
        m_Robot.m_telemetry.addData("CurrentX:", CurrentPos);


        m_Robot.m_telemetry.addData("PowerX:", Power);


    }

    @Override
    public boolean isFinished()
    {


        return false;
    }
}

package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.drive.OdometryComponent;
import org.firstinspires.ftc.teamcode.facade.drive.DriveSubsystem;
@Config

public class GotoX extends CommandBase {

    Telemetry m_telemetry;
        public static double P = 0.032d, I = 0.05d, D = 0.0002d, F = 0.0d;
    double CurrentPos = 0.0d;
    public double pmax = 1.0d;

    public double Tolerance = 4;

    public static double DefaultTolerance = 4;

    public GotoX()
    {
        Tolerance = DefaultTolerance;
    }
    public boolean isWithinTolerance()
    {
        double dist = Math.abs(m_Target - CurrentPos);
        if ( dist < Tolerance )
            return true;
        return false;
    }
    DriveSubsystem m_DriveSubsystem;
    PIDFController pid;

    public double m_Target = 0.0d;
    public double Power = 0.0d;

    public boolean active = true;
    RobotHardware m_Robot;
    public GotoX(double x, Telemetry telemetry, DriveSubsystem driveSubsystem, RobotHardware robot)
    {
        pid = new PIDFController(P, I, D, F);
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

    }

    public void increment(double inc)
    {
        m_Target -= inc;
        pid.setSetPoint(m_Target);
        Tolerance = DefaultTolerance;
    }

    public void increment(double inc, double tol)
    {
        m_Target -= inc;
        pid.setSetPoint(m_Target);
        Tolerance = tol;
    }
    @Override
    public void execute()
    {
        CurrentPos = OdometryComponent.X;

        if( isWithinTolerance()) {
            Power = 0.0d;
            return;
        }
        Power = pid.calculate(CurrentPos);

        if ( Power < -pmax )
            Power = -pmax;
        if ( Power > pmax)
            Power = pmax;
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

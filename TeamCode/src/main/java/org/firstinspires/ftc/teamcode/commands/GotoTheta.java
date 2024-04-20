package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.drive.OdometryComponent;
import org.firstinspires.ftc.teamcode.facade.drive.DriveSubsystem;
import org.opencv.core.Mat;

import java.lang.annotation.Target;
import java.sql.ParameterMetaData;

@Config
public class GotoTheta extends CommandBase {

    Telemetry m_telemetry;
    public static double P = 0.018, I= 0.1d, D = 0.0007d, F = 0.00d;

    public double pmax = 1.0d;
    public static double DefaultTolerance = 5;

    public static double Tolerance = 5;

    double CurrentPos = 0.0d;
    DriveSubsystem m_DriveSubsystem;
    PIDFController pid;

    public boolean isWithinTolerance()
    {
        double dist = Math.abs(m_Target - CurrentPos);
        if ( dist < Tolerance || 360 - dist < Tolerance)
            return true;
        return false;
    }
    public double m_Target = 0.0d;

    public double Power = 0.0d;
    RobotHardware m_Robot;

    public boolean active = true;
    public GotoTheta(double Theta, Telemetry telemetry, DriveSubsystem driveSubsystem, RobotHardware robot)
    {
        pid = new PIDFController(P, I, D, F);
        pid.setSetPoint(Theta);
        m_telemetry = telemetry;
        m_DriveSubsystem = driveSubsystem;
        m_Target = Theta;
        m_Robot = robot;
        active = true;
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {
        CurrentPos = OdometryComponent.Theta;
        double p2;
        double p1;
        if ( CurrentPos < m_Target) {
            p1 = Math.abs(CurrentPos - m_Target);
            p2 = 360 - p1;
        }
        else {
            p2 = Math.abs(CurrentPos - m_Target);
            p1 = 360 - p2;
        }

        if( isWithinTolerance()) {
            Power = 0.0d;
            return;
        }
        m_telemetry.addData("THETA target", m_Target);


        Power = Math.abs(pid.calculate(m_Target + Math.min(p1, p2)));
        if ( p1 < p2 )
            Power = -Power;

        if ( Power < -pmax )
            Power = -pmax;
        if ( Power > pmax)
            Power = pmax;
        m_Robot.m_telemetry.addData("TargetT:", m_Target);
        m_Robot.m_telemetry.addData("CurrentT:", CurrentPos);


        m_Robot.m_telemetry.addData("PowerT:", Power);
    }
    public void set(double pos)
    {
        pos =(pos + 360) % 360;
        m_Target = pos;
        pid.setSetPoint(pos);
    }
    @Override
    public boolean isFinished()
    {


        return false;
    }
}
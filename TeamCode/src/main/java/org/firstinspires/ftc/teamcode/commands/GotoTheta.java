package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

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
    public static double P = 0.049d, I= 0.1d, D = 0.00285d;

    public double pmax = 1.0d;
    public static double Tolerance = 6;

    double CurrentPos = 0.0d;
    DriveSubsystem m_DriveSubsystem;
    PIDController pid;

    public boolean isWithinTolerance()
    {
        double dist = Math.abs(m_Target - CurrentPos);
        if ( dist < Tolerance || 360 - dist < Tolerance)
            return true;
        return false;
    }
    private double m_Target = 0.0d;

    public double Power = 0.0d;
    RobotHardware m_Robot;

    public boolean active = true;
    public GotoTheta(double Theta, Telemetry telemetry, DriveSubsystem driveSubsystem, RobotHardware robot)
    {
        pid = new PIDController(P, I, D);
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
        Power = m_Target;
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

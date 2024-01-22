package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.drive.OdometryComponent;
import org.firstinspires.ftc.teamcode.facade.drive.DriveSubsystem;
@Config

public class GotoY extends CommandBase {

    Telemetry m_telemetry;
    public static double P = 0.055d, I = 0.1d, D = 0.0011d;
    double CurrentPos = 0.0d;
    public static double Tolerance = 3.5;

    public double pmax = 1.0d;


    public boolean isWithinTolerance()
    {
        double dist = Math.abs(m_Target - CurrentPos);
        if ( dist < Tolerance )
            return true;
        return false;
    }
    DriveSubsystem m_DriveSubsystem;
    PIDController pid;

    private double m_Target = 0.0d;

    public double Power = 0.0d;
    RobotHardware m_Robot;

    public GotoY(double y, Telemetry telemetry, DriveSubsystem driveSubsystem, RobotHardware robot)
    {
        pid = new PIDController(P, I, D);
        pid.setSetPoint(y);
        m_telemetry = telemetry;
        m_DriveSubsystem = driveSubsystem;
        m_Target = y;
        m_Robot = robot;
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {
        CurrentPos = OdometryComponent.Y;

        if( isWithinTolerance()) {
            Power = 0.0d;
            return;
        }
        Power = -pid.calculate(CurrentPos);

        if ( Power < -pmax )
            Power = -pmax;
        if ( Power > pmax)
            Power = pmax;
        m_Robot.m_telemetry.addData("TargetY:", m_Target);
        m_Robot.m_telemetry.addData("CurrentY:", CurrentPos);


        m_Robot.m_telemetry.addData("PowerY:", Power);

    }
    public void set(double pos)
    {
        m_Target = pos;
        pid.setSetPoint(pos);
    }
    @Override
    public boolean isFinished()
    {

        return false;
    }
}

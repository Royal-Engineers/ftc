package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.drive.OdometryComponent;
import org.firstinspires.ftc.teamcode.facade.drive.DriveSubsystem;

public class GotoY extends CommandBase {

    Telemetry m_telemetry;
    double P = 0.035d, I = 0.015d, D = 0.003d;

    DriveSubsystem m_DriveSubsystem;
    PIDController pid;

    private double m_Target = 0.0d;

    public double Power = 0.0d;
    RobotHardware m_Robot;

    boolean active = true;
    public GotoY(double y, Telemetry telemetry, DriveSubsystem driveSubsystem, RobotHardware robot)
    {
        pid = new PIDController(P, I, D);
        pid.setSetPoint(y);
        m_telemetry = telemetry;
        m_DriveSubsystem = driveSubsystem;
        m_Target = y;
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
        if ( !active ) {
            Power = 0.0d;
            return;
        }
        if( Math.abs(OdometryComponent.Y - m_Target) < 5.0d) {
            active = false;
            Power = 0.0d;
            return;
        }
        double CurrentPos = OdometryComponent.Y;
        Power = -pid.calculate(CurrentPos);
        m_Robot.m_telemetry.addData("TargetY:", m_Target);
        m_Robot.m_telemetry.addData("CurrentY:", CurrentPos);


        m_Robot.m_telemetry.addData("PowerY:", Power);

    }
    public void set(double pos)
    {
        m_Target = pos;
        pid.setSetPoint(pos);
        active = true;
    }
    @Override
    public boolean isFinished()
    {

        return false;
    }
}

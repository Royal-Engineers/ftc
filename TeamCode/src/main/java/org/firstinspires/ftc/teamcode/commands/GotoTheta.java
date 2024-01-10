package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.drive.OdometryComponent;
import org.firstinspires.ftc.teamcode.facade.drive.DriveSubsystem;
import org.opencv.core.Mat;

public class GotoTheta extends CommandBase {

    Telemetry m_telemetry;
    double P = 0.035d, I= 0.00d, D = 0.001d;
    DriveSubsystem m_DriveSubsystem;
    PIDController pid;

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
        if ( !active ) {
            Power = 0.0d;
            return;
        }
        if( Math.abs(OdometryComponent.Theta - m_Target) < 5.0d) {
            Power = 0.0d;
            active = false;
            return;
        }

            double CurrentPos = OdometryComponent.Theta;
        Power = Math.abs(pid.calculate(Math.min(Math.abs(CurrentPos), Math.abs(360 -CurrentPos))));
        m_Robot.m_telemetry.addData("TargetT:", m_Target);
        m_Robot.m_telemetry.addData("CurrentT:", CurrentPos);


        m_Robot.m_telemetry.addData("PowerT:", Power);
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

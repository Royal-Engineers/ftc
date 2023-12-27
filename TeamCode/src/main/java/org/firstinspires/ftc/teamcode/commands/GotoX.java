package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.facade.subsystems.OdometryComponent;
import org.firstinspires.ftc.teamcode.pipelines.SKN.DriveSubsystem;

public class GotoX extends CommandBase {

    Telemetry m_telemetry;
    double P = 0.20d, I = 0.0d, D = 0.0d;

    DriveSubsystem m_DriveSubsystem;
    PIDController pid;

    public GotoX(double x, Telemetry telemetry, DriveSubsystem driveSubsystem)
    {
        pid = new PIDController(P, I, D);
        pid.setSetPoint(x);
        m_telemetry = telemetry;
        m_DriveSubsystem = driveSubsystem;
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
    }

    @Override
    public boolean isFinished()
    {

        return false;
    }
}

package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.facade.drive.DriveSubsystem;

public class KeepPosition extends CommandBase {

    public static boolean isActive = true;

    public GotoX XCommand;
    public GotoY YCommand;
    public GotoTheta TCommand;

    DriveSubsystem m_DriveSubsystem;
    public KeepPosition(GotoX x, GotoY y, GotoTheta T, DriveSubsystem driveSubsystem)
    {
        XCommand = x;
        YCommand = y;
        TCommand = T;
        isActive = true;
        m_DriveSubsystem = driveSubsystem;
    }

    @Override
    public  void initialize()
    {
        XCommand.initialize();
        YCommand.initialize();
        TCommand.initialize();
    }
    double TX = 0.0d, TY = 0.0d, TT = 0.0d;
    public void SetTheta(double T)
    {
        TCommand.set(T);
    }

    public void SetX(double T)
    {
        XCommand.set(T);
    }
    public void SetY(double T)
    {
        YCommand.set(T);
    }

    public void SetAll(double x, double y, double theta)
    {
        TCommand.set(theta);
        XCommand.set(x);
        YCommand.set(y);
        TCommand.Tolerance = GotoTheta.DefaultTolerance;
        XCommand.Tolerance = GotoX.DefaultTolerance;
        YCommand.Tolerance = GotoY.DefaultTolerance;
    }

    public void SetAll(double x, double y, double theta, double px, double py, double pt)
    {
        TCommand.set(theta);
        XCommand.set(x);
        YCommand.set(y);
        TCommand.pmax = pt;
        YCommand.pmax = py;
        XCommand.pmax = px;
        TCommand.Tolerance = pt;
        XCommand.Tolerance = px;
        YCommand.Tolerance = py;
    }

    @Override public void execute()
    {
        if ( isActive == false )
            return;
        XCommand.execute();
        YCommand.execute();
        TCommand.execute();

        m_DriveSubsystem.UpdateAuto(XCommand.Power, YCommand.Power, TCommand.Power);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

}

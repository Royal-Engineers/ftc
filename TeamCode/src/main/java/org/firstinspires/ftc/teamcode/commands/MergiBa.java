package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.facade.drive.DriveSubsystem;

public class MergiBa extends SequentialCommandGroup {
    public MergiBa(double x, double y, double t, KeepPosition PositionCommand)
    {
        super(
                new InstantCommand(()->{PositionCommand.SetAll(x, y, t);}),
                new WaitUntilCommand(()-> {
                    return PositionCommand.XCommand.isWithinTolerance()  && PositionCommand.YCommand.isWithinTolerance()
                            && PositionCommand.m_DriveSubsystem.InTolerance;
                }));


    }

    public MergiBa(double x, double y, double t, KeepPosition PositionCommand, double px, double py, double pt)
    {
        super(
                new InstantCommand(()->{PositionCommand.SetAll(x, y, t, px, py, pt);}),
                new WaitUntilCommand(()-> {
                    return PositionCommand.XCommand.isWithinTolerance()  && PositionCommand.YCommand.isWithinTolerance()
                            && PositionCommand.TCommand.isWithinTolerance();
                }));


    }
}

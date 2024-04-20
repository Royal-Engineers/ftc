package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.facade.drive.DriveSubsystem;

public class FollowPath extends SequentialCommandGroup {
    public FollowPath(double x, double y, double t, KeepPosition PositionCommand)
    {
        super(
                new InstantCommand(()->{PositionCommand.SetAll(x, y, t);}),
                new WaitUntilCommand(()-> {
                    return PositionCommand.XCommand.isWithinTolerance()  && PositionCommand.YCommand.isWithinTolerance()
                            && PositionCommand.TCommand.isWithinTolerance();
                }));


    }

    public FollowPath(double x, double y, double t, KeepPosition PositionCommand, double tx, double ty, double tt)
    {
        super(
                new InstantCommand(()->{PositionCommand.SetAll(x, y, t, tx, ty, tt);}),
                new WaitUntilCommand(()-> {
                    return PositionCommand.XCommand.isWithinTolerance()  && PositionCommand.YCommand.isWithinTolerance()
                            && PositionCommand.TCommand.isWithinTolerance();
                }));


    }
}

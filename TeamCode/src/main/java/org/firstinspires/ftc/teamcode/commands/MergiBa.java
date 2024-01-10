package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

public class MergiBa extends SequentialCommandGroup {
    public MergiBa(double x, double y, double t, KeepPosition PositionCommand)
    {
        super(
                new InstantCommand(()->{PositionCommand.SetAll(x, y, t);}),
                new WaitUntilCommand(()-> {
                    return PositionCommand.XCommand.active == false && PositionCommand.YCommand.active == false
                            && PositionCommand.TCommand.active == false;
                }),
                new InstantCommand(()->{PositionCommand.SetAll(x, y, t);}));


    }
}

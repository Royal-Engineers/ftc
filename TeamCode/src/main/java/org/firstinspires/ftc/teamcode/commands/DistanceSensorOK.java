package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

public class DistanceSensorOK extends CommandBase {
    public DistanceSensorOK()
    {

    }

    @Override
    public void initialize()
    {

    }

    @Override
    public boolean isFinished()
    {
        return DistanceSensorCommand.ok;
    }
}

package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.commandBaseTeleOp;

public class ToggleStackON extends CommandBase {

    public ToggleStackON(){}

    @Override public void initialize(){
        commandBaseTeleOp.ReachForStack = true;
    }

    @Override public boolean isFinished(){
        return true;
    }
}

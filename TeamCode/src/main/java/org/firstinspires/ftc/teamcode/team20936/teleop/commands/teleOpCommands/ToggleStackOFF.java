package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.commandBaseTeleOp;

public class ToggleStackOFF extends CommandBase {

    public ToggleStackOFF(){}

    @Override public void initialize(){
        commandBaseTeleOp.ReachForStack = false;
    }

    @Override public boolean isFinished(){
        return true;
    }
}

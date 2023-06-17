package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.depositSubsystem;

public class retract extends CommandBase {

    depositSubsystem m_depositSubsystem;

    public retract (depositSubsystem subsystem) {
        m_depositSubsystem = subsystem;
    }

    @Override
    public void initialize() { m_depositSubsystem.retract(); }

    @Override
    public boolean isFinished() { return true; }

}

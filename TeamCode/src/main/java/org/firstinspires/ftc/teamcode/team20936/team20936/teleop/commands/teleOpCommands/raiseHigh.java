package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.depositSubsystem;

public class raiseHigh extends CommandBase {

    depositSubsystem m_depositSubsystem;

    public raiseHigh(depositSubsystem subsystem) {
        m_depositSubsystem = subsystem;
        addRequirements(m_depositSubsystem);
    }

    @Override
    public void initialize() { m_depositSubsystem.raiseHigh(); }

    @Override
    public boolean isFinished() { return true; }

}

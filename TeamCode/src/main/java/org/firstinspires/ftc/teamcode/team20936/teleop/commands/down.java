package org.firstinspires.ftc.teamcode.team20936.teleop.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.revSubsystem;

public class down extends CommandBase {

    revSubsystem m_revSubsystem;

    public down (revSubsystem subsystem) {
        m_revSubsystem = subsystem;
        addRequirements(m_revSubsystem);
    }

    @Override
    public void initialize() { m_revSubsystem.down(); }

    @Override
    public boolean isFinished() { return true; }

}

package org.firstinspires.ftc.teamcode.team20936.teleop.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.revSubsystem;

public class up extends CommandBase {

    revSubsystem m_revSubsystem;

    public up (revSubsystem subsystem) {
        m_revSubsystem = subsystem;
        addRequirements(m_revSubsystem);
    }

    @Override
    public void initialize() { m_revSubsystem.up(); }

    @Override
    public boolean isFinished() { return true; }

}

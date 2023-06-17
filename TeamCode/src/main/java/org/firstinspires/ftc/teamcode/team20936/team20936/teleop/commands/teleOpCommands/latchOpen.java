package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.depositSubsystem;

public class latchOpen extends CommandBase {

    depositSubsystem m_depositSubsystem;

    public latchOpen(depositSubsystem subsystem) {
        m_depositSubsystem = subsystem;
        addRequirements(m_depositSubsystem);
    }

    @Override
    public void initialize() { m_depositSubsystem.latchOpen(); }

    @Override
    public boolean isFinished() { return true; }

}

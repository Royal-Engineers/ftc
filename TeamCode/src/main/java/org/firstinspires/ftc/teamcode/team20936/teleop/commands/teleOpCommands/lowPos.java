package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;

public class lowPos extends CommandBase {

    private final intakeSubsystem m_intakeSubsystem;

    public lowPos(intakeSubsystem intakeSubsystem)
    {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
    }
    @Override
    public void initialize()
    {
        m_intakeSubsystem.lowPos();
    }

    @Override
    public boolean isFinished() { return true; }
}

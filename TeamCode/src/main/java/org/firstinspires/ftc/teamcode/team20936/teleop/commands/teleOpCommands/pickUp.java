package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;

public class pickUp extends CommandBase {
    private final intakeSubsystem m_intakeSubsystem;

    public pickUp(intakeSubsystem subsystem)
    {
        m_intakeSubsystem = subsystem;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize()
    {
        m_intakeSubsystem.pickUp();
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}

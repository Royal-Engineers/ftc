package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.stackCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;

public class stack_3rdcone extends CommandBase {

    intakeSubsystem m_intakeSubsystem;

    public stack_3rdcone (intakeSubsystem subsystem) {
        m_intakeSubsystem = subsystem;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() { m_intakeSubsystem.stack_3rdcone(); }

    @Override
    public boolean isFinished() { return true; }
}

package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.stackCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;

public class setToggleStack extends CommandBase {

    intakeSubsystem m_intakeSubsystem;
    private boolean value;

    public setToggleStack(intakeSubsystem subsystem, boolean value) {
        m_intakeSubsystem = subsystem;
        this.value = value;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() { m_intakeSubsystem.setToggleStack(value); }

    @Override
    public boolean isFinished() { return true; }

}

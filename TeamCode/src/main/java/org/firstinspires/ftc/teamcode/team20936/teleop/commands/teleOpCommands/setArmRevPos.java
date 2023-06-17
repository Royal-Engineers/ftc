package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;

public class setArmRevPos extends CommandBase {

    intakeSubsystem m_intakeSubsystem;
    int target;

    public setArmRevPos (intakeSubsystem subsystem, int target) {
        m_intakeSubsystem = subsystem;
        this.target = target;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() { m_intakeSubsystem.setArmRevPos(target); }

    @Override
    public boolean isFinished() { return true; }

}

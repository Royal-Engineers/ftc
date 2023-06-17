package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;

import com.arcrobotics.ftclib.command.CommandBase;

public class clawGrab extends CommandBase{

    private final intakeSubsystem m_intakeSubsystem;

    public clawGrab(intakeSubsystem subsystem) {
        m_intakeSubsystem = subsystem;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.grab();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

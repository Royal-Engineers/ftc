package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;

public class RecoverCone extends CommandBase {

    private intakeSubsystem m_intakeSubsystem;

    public RecoverCone(intakeSubsystem IntakeSubsystem)
    {
        m_intakeSubsystem = IntakeSubsystem;
    }

    @Override public void initialize(){
        m_intakeSubsystem.setWristPos(0.7);
        m_intakeSubsystem.setWristRevPos(0.426);
    }
    @Override
    public boolean isFinished()
    {
        return true;
    }
}

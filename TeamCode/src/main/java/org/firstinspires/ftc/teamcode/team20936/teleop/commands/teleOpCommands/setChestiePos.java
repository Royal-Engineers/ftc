package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;

public class setChestiePos extends CommandBase {

    private intakeSubsystem m_intakeSubsystem;
    private double m_target;

    public setChestiePos(intakeSubsystem IntakeSubsystem, double target){
        m_intakeSubsystem = IntakeSubsystem;
        addRequirements(IntakeSubsystem);

        this.m_target = target;
    }
    @Override public void initialize(){
        m_intakeSubsystem.setChestiePos(m_target);
    }

    @Override public boolean isFinished(){
        return true;
    }
}

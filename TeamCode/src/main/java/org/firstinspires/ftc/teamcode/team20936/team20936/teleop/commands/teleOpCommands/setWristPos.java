package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;

public class setWristPos extends CommandBase {

    intakeSubsystem m_intakeSubsystem;
    double target;

    public setWristPos(intakeSubsystem subsystem, double target) {
        m_intakeSubsystem = subsystem;
        this.target = target;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() { m_intakeSubsystem.setWristPos(target); }

    @Override
    public boolean isFinished() { return true; }

}

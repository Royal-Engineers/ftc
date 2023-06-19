package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;

public class setManualRevPos extends CommandBase {

    intakeSubsystem m_intakeSubsystem;
    private double input;

    public setManualRevPos(intakeSubsystem subsystem, double input) {
        m_intakeSubsystem = subsystem;
        this.input = input;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() { m_intakeSubsystem.setManualRevPos(input); }

    @Override
    public boolean isFinished() { return true; }

}

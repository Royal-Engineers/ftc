package org.firstinspires.ftc.teamcode.team20936.teleop.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;

public class setManualArmRevPos extends CommandBase {

    intakeSubsystem m_intakeSubsystem;
    double target;

    public setManualArmRevPos(intakeSubsystem subsystem, double target) {
        m_intakeSubsystem = subsystem;
        this.target = target;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() { m_intakeSubsystem.setManualArmRevPos(target); }

    @Override
    public boolean isFinished() { return true; }

}

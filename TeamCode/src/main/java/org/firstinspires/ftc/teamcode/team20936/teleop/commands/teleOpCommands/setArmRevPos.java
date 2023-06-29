package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;

public class setArmRevPos extends CommandBase {

    intakeSubsystem m_intakeSubsystem;
    int target;
    private boolean hasPow = false;
    private double power;

    public setArmRevPos (intakeSubsystem subsystem, int target) {
        hasPow = false;
        m_intakeSubsystem = subsystem;
        this.target = target;
        addRequirements(m_intakeSubsystem);
    }


    public setArmRevPos (intakeSubsystem subsystem, int target, double pow) {
        hasPow = true;
        power = pow;
        m_intakeSubsystem = subsystem;
        this.target = target;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {
        if ( hasPow == false )
        m_intakeSubsystem.setArmRevPos(target);
        else
            m_intakeSubsystem.setArmRevPos(target, power);
    }

    @Override
    public boolean isFinished() { return true; }

}

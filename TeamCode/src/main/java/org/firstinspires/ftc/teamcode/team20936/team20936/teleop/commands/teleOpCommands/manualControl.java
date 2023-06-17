package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.depositSubsystem;

public class manualControl extends CommandBase {

    depositSubsystem m_depositSubsystem;
    double target;

    public manualControl(depositSubsystem subsystem, double target)
    {
        m_depositSubsystem = subsystem;
        this.target = target;
        addRequirements(m_depositSubsystem);
    }

    @Override
    public void initialize() { m_depositSubsystem.manualControl(target); }

    @Override
    public boolean isFinished() { return true; }

}

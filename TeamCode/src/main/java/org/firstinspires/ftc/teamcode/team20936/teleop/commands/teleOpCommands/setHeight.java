package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.depositSubsystem;

public class setHeight extends CommandBase {

    depositSubsystem m_depositSubsystem;
    private int height;

    public setHeight(depositSubsystem subsystem, int height) {
        m_depositSubsystem = subsystem;
        this.height = height;
        addRequirements(m_depositSubsystem);
    }

    @Override
    public void initialize() { m_depositSubsystem.setHeight(height); }

    @Override
    public boolean isFinished() { return true; }

}

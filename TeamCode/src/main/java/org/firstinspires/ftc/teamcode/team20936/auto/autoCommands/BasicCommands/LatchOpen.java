package org.firstinspires.ftc.teamcode.team20936.auto.autoCommands.BasicCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.auto.subsystems.Singleton_AutoSubsystem;
public class LatchOpen extends CommandBase{
    private final Singleton_AutoSubsystem m_subsystem;
    private final double LatchOpenPose = 0.1;
    public LatchOpen()
    {
        m_subsystem = Singleton_AutoSubsystem.getInstance();
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.setLatchPos(LatchOpenPose);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

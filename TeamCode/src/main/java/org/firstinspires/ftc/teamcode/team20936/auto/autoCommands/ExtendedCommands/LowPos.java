package org.firstinspires.ftc.teamcode.team20936.auto.autoCommands.ExtendedCommands;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.team20936.auto.subsystems.Singleton_AutoSubsystem;

import kotlin.OptionalExpectation;

public class LowPos extends CommandBase{
    private final Singleton_AutoSubsystem m_subsystem;
    private final double WristOne =0.163, WristRevFlat = 0.413;

    public LowPos(){
        m_subsystem = Singleton_AutoSubsystem.getInstance();
        addRequirements(m_subsystem);
    }
    @Override
    public void initialize(){
      m_subsystem.setWristRevPos(WristRevFlat);
      m_subsystem.setWristPos(WristOne);
      m_subsystem.setArmRevPos(-80, 0.75);
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}

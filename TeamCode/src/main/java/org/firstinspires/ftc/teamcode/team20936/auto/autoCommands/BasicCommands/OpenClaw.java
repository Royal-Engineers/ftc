package org.firstinspires.ftc.teamcode.team20936.auto.autoCommands.BasicCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.auto.subsystems.Singleton_AutoSubsystem;

public class OpenClaw extends CommandBase{
    private final Singleton_AutoSubsystem autoSubsystem;
    private final double Claw_OpenPosition = 0.22;

    public OpenClaw()
    {
        autoSubsystem = Singleton_AutoSubsystem.getInstance();
        addRequirements(autoSubsystem);
    }

    @Override
    public void initialize() {
        autoSubsystem.setClawPos(Claw_OpenPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

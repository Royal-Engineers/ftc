package org.firstinspires.ftc.teamcode.team20936.auto.autoCommands.BasicCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.auto.subsystems.Singleton_AutoSubsystem;

public class CloseClaw extends CommandBase{
    private final Singleton_AutoSubsystem autoSubsystem;
    private final double Claw_ClosedPosition = 0.00;
    public CloseClaw()
    {
        autoSubsystem = Singleton_AutoSubsystem.getInstance();
        addRequirements(autoSubsystem);
    }

    @Override
    public void initialize() {
        autoSubsystem.setClawPos(Claw_ClosedPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}


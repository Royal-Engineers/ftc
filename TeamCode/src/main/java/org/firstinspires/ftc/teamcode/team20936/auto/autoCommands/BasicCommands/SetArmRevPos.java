package org.firstinspires.ftc.teamcode.team20936.auto.autoCommands.BasicCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.apache.commons.math3.analysis.function.Sin;
import org.firstinspires.ftc.teamcode.team20936.auto.subsystems.Singleton_AutoSubsystem;

public class SetArmRevPos extends CommandBase{
    private final Singleton_AutoSubsystem autoSubsystem;
    private final int target;
    private boolean CPower = false;

    private double power = 0.00;
    public SetArmRevPos( int target)
    {

        CPower = false;

        autoSubsystem = Singleton_AutoSubsystem.getInstance();
        addRequirements(autoSubsystem);

        this.target = target;
    }

    public SetArmRevPos(int target, double power)
    {
        CPower = true;

        autoSubsystem = Singleton_AutoSubsystem.getInstance();
        addRequirements(autoSubsystem);

        this.power = power;
        this.target = target;
    }

    @Override
    public void initialize() {
        if ( CPower)
            autoSubsystem.setArmRevPos(target, this.power);
        else if ( target < -350 )
            autoSubsystem.setArmRevPos(target, 0.25);
        else if (target < -100)
            autoSubsystem.setArmRevPos(target, 0.6);
        else
            autoSubsystem.setArmRevPos(target, 1);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

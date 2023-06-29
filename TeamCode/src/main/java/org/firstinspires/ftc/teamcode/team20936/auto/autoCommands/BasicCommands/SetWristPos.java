package org.firstinspires.ftc.teamcode.team20936.auto.autoCommands.BasicCommands;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.team20936.auto.subsystems.Singleton_AutoSubsystem;

public class SetWristPos extends CommandBase{
    private final Singleton_AutoSubsystem autoSubsystem;
    private final double target;
    public SetWristPos(double target)
    {
        autoSubsystem = Singleton_AutoSubsystem.getInstance();
        addRequirements(autoSubsystem);

        this.target = target;
    }

    @Override
    public void initialize() {
        autoSubsystem.setWristPos(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

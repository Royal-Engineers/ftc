package org.firstinspires.ftc.teamcode.team20936.auto.autoCommands.BasicCommands;
import static android.os.SystemClock.sleep;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.auto.subsystems.Singleton_AutoSubsystem;

public class SorinMakesBetterWaitCommands extends CommandBase {
     private final Singleton_AutoSubsystem m_subsystem;
     private int target;
     public SorinMakesBetterWaitCommands(int target) {
         m_subsystem = Singleton_AutoSubsystem.getInstance();
         addRequirements(m_subsystem);
         this.target = target;
     }

     @Override
     public void initialize(){
         sleep(target);
     }
     @Override
    public boolean isFinished() {
         return true;
     }
}

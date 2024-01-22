package org.firstinspires.ftc.teamcode.commands.LiftCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.facade.RobotHardware;

public class LiftMiddle extends ParallelCommandGroup {
    public LiftMiddle(RobotHardware m_Robot)
    {
                super(
                        new InstantCommand(()->{m_Robot.m_Lift.SetTargetPosition(300, 0.8);}),
                        new InstantCommand(()->{m_Robot.m_Bar.SetPosition(0.57);}),
                        new InstantCommand(()->{m_Robot.m_ClawAngleServo.setPosition(RobotHardware.s_ScoringClawAngle);})

        );
    }
}

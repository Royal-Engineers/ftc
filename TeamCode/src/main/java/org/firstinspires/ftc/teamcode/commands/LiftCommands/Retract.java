package org.firstinspires.ftc.teamcode.commands.LiftCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.intake.Lift;

public class Retract extends ParallelCommandGroup {

    public Retract(RobotHardware m_Robot)
    {
        super(
                new SequentialCommandGroup(
                        new InstantCommand(()->{m_Robot.m_Claw.setPosition(RobotHardware.s_ClawTransfer);}),
                        new WaitCommand(500),
                        new InstantCommand(()->{m_Robot.m_Bar.SetPosition(0.0);}),
                        new InstantCommand(()->{m_Robot.m_ClawAngleServo.setPosition(RobotHardware.s_IdleClawAngle);}),
                 new WaitCommand(400),
                 new InstantCommand(()->{m_Robot.m_Lift.SetStatePosition(Lift.e_LiftPosition.LowPos);})

                        )

        );
    }
}

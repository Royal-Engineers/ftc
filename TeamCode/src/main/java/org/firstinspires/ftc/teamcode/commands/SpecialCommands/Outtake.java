package org.firstinspires.ftc.teamcode.commands.SpecialCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.facade.RobotHardware;

public class Outtake extends SequentialCommandGroup {

    public Outtake(RobotHardware m_Robot)
    {
        super(  new InstantCommand(()->
                {
                m_Robot.m_Bar.SetPosition(0.3);
                }),
                new WaitCommand(200),
                new InstantCommand(()->
                {
                    m_Robot.m_ServoIntake.setPosition(RobotHardware.OuttakePos);
                    m_Robot.motorIntake.setPower(-0.8);
                }),
                new WaitCommand(2000),
                new InstantCommand(()->
                {
                    m_Robot.m_ServoIntake.setPosition(0.52);
                    m_Robot.motorIntake.setPower(0.0);
                }),
                new WaitCommand(200),
                new InstantCommand(()->{m_Robot.m_Claw.setPosition(RobotHardware.s_ClawOpenPos);}),
                new WaitCommand(800),
                new InstantCommand(()->{m_Robot.m_Bar.SetPosition(0.0);}),
                new InstantCommand(()->{m_Robot.m_ClawAngleServo.setPosition(RobotHardware.s_IdleClawAngle);}
        ));
    }
}

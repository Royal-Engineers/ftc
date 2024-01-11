package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.intake.BombasticLift;

@Config
public class Transfer extends SequentialCommandGroup {

    public static double clawAngle1 = 0.81;
    public static double bar1 = -0.076;
    public static double clawAngle2 = 0.82;
    public static double bar2 = -0.08;
    public static double clawAngle3 = 0.81;
    public static double bar3 = -0.08;

    public static double clawAngle4 = 0.83;
    public static double bar4 = -0.09;

    public static double clawAngle5 = 0.84;
    public static double bar5 = -0.109;

    public static double clawAngle6 = 0.835;
    public static double bar6 = -0.155;

    public static double clawAngle7 = 0.89;
    public static double bar7 = -0.17;

    public Transfer(RobotHardware m_Robot)
    {
        super(
                new BombasticLiftPos(m_Robot, m_Robot.m_Lift, BombasticLift.e_LiftPosition.LowPos),
                new InstantCommand(()->{m_Robot.m_Gyara.setPosition(RobotHardware.s_ClawTransfer);}),
                new InstantCommand(()->{m_Robot.m_GyaraBomba.setPosition(clawAngle1);}),
                new InstantCommand(()->{m_Robot.m_BombaSexy.SetPosition(bar1);}),
                new WaitCommand(20),
                new InstantCommand(()->{m_Robot.m_GyaraBomba.setPosition(clawAngle2);}),
                new InstantCommand(()->{m_Robot.m_BombaSexy.SetPosition(bar2);}),
                new WaitCommand(5),
                new InstantCommand(()->{m_Robot.m_GyaraBomba.setPosition(clawAngle3);}),
                new InstantCommand(()->{m_Robot.m_BombaSexy.SetPosition(bar3);}),
                new WaitCommand(5),
                new InstantCommand(()->{m_Robot.m_GyaraBomba.setPosition(clawAngle4);}),
                new InstantCommand(()->{m_Robot.m_BombaSexy.SetPosition(bar4);}),
                new WaitCommand(5),
                new InstantCommand(()->{m_Robot.m_GyaraBomba.setPosition(clawAngle5);}),
                new InstantCommand(()->{m_Robot.m_BombaSexy.SetPosition(bar5);}),
                new WaitCommand(5),
                new InstantCommand(()->{m_Robot.m_GyaraBomba.setPosition(clawAngle6);}),
                new InstantCommand(()->{m_Robot.m_BombaSexy.SetPosition(bar6);}),
                new WaitCommand(5),
                new InstantCommand(()->{m_Robot.m_GyaraBomba.setPosition(clawAngle7);}),
                new InstantCommand(()->{m_Robot.m_BombaSexy.SetPosition(bar7);}),
                new WaitCommand(5),
                new InstantCommand(()->{m_Robot.m_Gyara.setPosition(RobotHardware.s_ClawlClosedPos);}),
                new WaitCommand(25),
                new InstantCommand(()->{m_Robot.m_BombaSexy.SetPosition(0.0);}),
                new InstantCommand(()->{m_Robot.m_GyaraBomba.setPosition(RobotHardware.s_IdleClawAngle);})

        );
    }
}

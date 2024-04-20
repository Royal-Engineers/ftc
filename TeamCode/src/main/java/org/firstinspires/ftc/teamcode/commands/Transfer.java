package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.intake.Lift;

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

    public static double clawAngle5 = 0.85;
    public static double bar5 = -0.129;

    public static double clawAngle6 = 0.835;
    public static double bar6 = -0.155;

    public static double clawAngle7 = 0.91;
    public static double bar7 = -0.17;

    public Transfer(RobotHardware m_Robot)
    {
        super(
                new BombasticLiftPos(m_Robot, m_Robot.m_Lift, Lift.e_LiftPosition.LowPos),
                new InstantCommand(()->{m_Robot.m_Claw2.setPosition(RobotHardware.s_Claw2Transfer);}),
                new InstantCommand(()->{m_Robot.m_Claw.setPosition(RobotHardware.s_ClawTransfer);}),
                new InstantCommand(()->{m_Robot.m_ClawAngleServo.setPosition(clawAngle1);}),
                new InstantCommand(()->{m_Robot.m_Bar.SetPosition(bar1);}),
                new WaitCommand(20),
                new InstantCommand(()->{m_Robot.m_ClawAngleServo.setPosition(clawAngle2);}),
                new InstantCommand(()->{m_Robot.m_Bar.SetPosition(bar2);}),
                //new WaitCommand(5),
                //new InstantCommand(()->{m_Robot.m_GyaraBomba.setPosition(clawAngle3);}),
                //new InstantCommand(()->{m_Robot.m_BombaSexy.SetPosition(bar3);}),
                new WaitCommand(5),
                new InstantCommand(()->{m_Robot.m_ClawAngleServo.setPosition(clawAngle4);}),
                new InstantCommand(()->{m_Robot.m_Bar.SetPosition(bar4);}),
                new WaitCommand(5),
                new InstantCommand(()->{m_Robot.m_ClawAngleServo.setPosition(clawAngle5);}),
                new InstantCommand(()->{m_Robot.m_Bar.SetPosition(bar5);}),
                new WaitCommand(5),
                //new InstantCommand(()->{m_Robot.m_GyaraBomba.setPosition(clawAngle6);}),
                //new InstantCommand(()->{m_Robot.m_BombaSexy.SetPosition(bar6);}),
                //new WaitCommand(5),
                new InstantCommand(()->{m_Robot.m_ClawAngleServo.setPosition(clawAngle7);}),
                new InstantCommand(()->{m_Robot.m_Bar.SetPosition(bar7);}),
                new WaitCommand(5),
                new InstantCommand(()->{m_Robot.m_Claw.setPosition(RobotHardware.s_ClawlClosedPos);}),
                new WaitCommand(150),
                new InstantCommand(()->{m_Robot.m_Bar.SetPosition(0.0);}),
                new InstantCommand(()->{m_Robot.m_ClawAngleServo.setPosition(RobotHardware.s_IdleClawAngle);}),
                new WaitCommand(250),
                new InstantCommand(()->{m_Robot.m_Claw2.setPosition(RobotHardware.s_IdleClaw2Angle);})

                );
    }
}

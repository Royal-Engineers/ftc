package org.firstinspires.ftc.teamcode.facade.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.Transfer;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.intake.Lift;
import org.firstinspires.ftc.teamcode.facade.interfaces.i_gamepad;

@Config
public class controller2 extends i_gamepad {
    public static double ba = 0.0d;
    public controller2(Gamepad gamepad, RobotHardware robot){
        super(gamepad, robot);
    }
int GyatLevel = 0;
    @Override
    public void initialize()
    {

        controller.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(()-> {
                    if (Math.abs(m_Robot.motorIntake.getPower()) < 0.01)
                        m_Robot.motorIntake.setPower(0.8f);
                    else
                        m_Robot.motorIntake.setPower(0.0f);
                })
        );

        controller.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(()-> {
                    if (Math.abs(m_Robot.motorIntake.getPower()) < 0.01)
                        m_Robot.motorIntake.setPower(-0.8f);
                    else
                        m_Robot.motorIntake.setPower(0.0f);
                })
        );

        controller.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                        new Transfer(m_Robot)
        );

        controller.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).toggleWhenPressed(
                new InstantCommand(()->
                {
                    m_Robot.m_Lift.SetTargetPosition(1400, 1.0);
                    m_Robot.m_Bar.SetPosition(0.3);
                }),
                new InstantCommand(()->
                {

                    m_Robot.m_Lift.SetTargetPosition(750, 1.0);
                })
        );

        controller.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(()->{
                    m_Robot.m_ServoAvion.setPosition(0.1);
                })
        );
        controller.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(()->{m_Robot.m_ClawAngleServo.setPosition(0.81);}),
                        new InstantCommand(()->{m_Robot.m_Bar.SetPosition(-0.076);}),
                        new WaitCommand(100),
                        new InstantCommand(()->{
                            m_Robot.m_ClawAngleServo.setPosition(RobotHardware.s_IdleClawAngle);
                            m_Robot.m_Bar.SetPosition(0.0d);
                        })
                ));


    }

    static double PosIntake = 0.86;
    @Override
    public void update() {
        if (gamepad.left_stick_y < -MinPush )
            sg_CommandScheduler.schedule(
                    new InstantCommand(()->{
                            m_Robot.m_Bar.SetPosition(m_Robot.m_Bar.GetPosition() - 0.01);}
            ));
        else if ( gamepad.left_stick_y > MinPush )
            sg_CommandScheduler.schedule(
                    new InstantCommand(()->{
                        m_Robot.m_Bar.SetPosition(m_Robot.m_Bar.GetPosition() + 0.01);}
                    ));
/*
        if (gamepad.left_trigger > MinPush )
            sg_CommandScheduler.schedule(
                    new InstantCommand(()->{
                        PosIntake-=0.01d;
                        if ( PosIntake < 0)
                            PosIntake = 0;
                        m_Robot.m_ServoIntake.setPosition(PosIntake);}
                    ));
        else if ( gamepad.right_trigger > MinPush )
            sg_CommandScheduler.schedule(
                    new InstantCommand(()->{
                        PosIntake+=0.01d;
                        if ( PosIntake > 1)
                            PosIntake = 1;
                        m_Robot.m_ServoIntake.setPosition(PosIntake);}
                    ));*/
    }


}



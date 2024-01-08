package org.firstinspires.ftc.teamcode.facade.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.intake.BombasticLift;
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
                new InstantCommand(()->{m_Robot.m_Gyara.setPosition(ba);}));



    }

    @Override
    public void update() {
        if (gamepad.left_stick_y < -MinPush )
            sg_CommandScheduler.schedule(
                    new InstantCommand(()->{
                            m_Robot.m_BombaSexy.SetPosition(m_Robot.m_BombaSexy.GetPosition() - 0.01);}
            ));
        else if ( gamepad.left_stick_y > MinPush )
            sg_CommandScheduler.schedule(
                    new InstantCommand(()->{
                        m_Robot.m_BombaSexy.SetPosition(m_Robot.m_BombaSexy.GetPosition() + 0.01);}
                    ));

        if (gamepad.left_trigger > MinPush )
            sg_CommandScheduler.schedule(
                    new InstantCommand(()->{
                        m_Robot.m_GyaraBomba.setPosition(m_Robot.m_GyaraBomba.getPosition() - 0.01);}
                    ));
        else if ( gamepad.right_trigger > MinPush )
            sg_CommandScheduler.schedule(
                    new InstantCommand(()->{
                        m_Robot.m_GyaraBomba.setPosition(m_Robot.m_GyaraBomba.getPosition() + 0.01);}
                    ));
        m_telemetry.addData("GYAT LEVEL", GyatLevel);
    }


}



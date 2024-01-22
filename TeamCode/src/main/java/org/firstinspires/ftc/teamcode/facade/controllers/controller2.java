package org.firstinspires.ftc.teamcode.facade.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
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

        controller.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(()->{m_Robot.m_Lift.SetTargetPosition(200.0d, Lift.s_SafetyPower);})
        );

        controller.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(()->{m_Robot.m_Lift.SetTargetPosition(400.0d, Lift.s_SafetyPower);})
        );

        controller.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(()->{m_Robot.m_Lift.SetTargetPosition(600.0d, Lift.s_SafetyPower);})
        );

        controller.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(()->{m_Robot.m_Lift.SetTargetPosition(799.0d, Lift.s_SafetyPower);})
        );


    }

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

        if (gamepad.left_trigger > MinPush )
            sg_CommandScheduler.schedule(
                    new InstantCommand(()->{
                        m_Robot.m_ServoIntake.setPosition(m_Robot.m_ServoIntake.getPosition() - 0.01);}
                    ));
        else if ( gamepad.right_trigger > MinPush )
            sg_CommandScheduler.schedule(
                    new InstantCommand(()->{
                        m_Robot.m_ServoIntake.setPosition(m_Robot.m_ServoIntake.getPosition() + 0.01);}
                    ));
    }


}



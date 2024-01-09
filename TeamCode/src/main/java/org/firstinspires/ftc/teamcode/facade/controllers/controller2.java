package org.firstinspires.ftc.teamcode.facade.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.BombasticLiftPos;
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
                new SequentialCommandGroup(
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
                        new WaitCommand(70),
                        new InstantCommand(()->{m_Robot.m_Gyara.setPosition(RobotHardware.s_ClawlClosedPos);}),
                        new WaitCommand(50),
                        new InstantCommand(()->{m_Robot.m_BombaSexy.SetPosition(0.0);}),
                        new InstantCommand(()->{m_Robot.m_GyaraBomba.setPosition(RobotHardware.s_IdleClawAngle);})


                )
        );

        controller.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(()->{m_Robot.m_Lift.SetTargetPosition(200.0d, BombasticLift.s_SafetyPower);})
        );

        controller.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(()->{m_Robot.m_Lift.SetTargetPosition(400.0d, BombasticLift.s_SafetyPower);})
        );

        controller.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(()->{m_Robot.m_Lift.SetTargetPosition(600.0d, BombasticLift.s_SafetyPower);})
        );

        controller.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(()->{m_Robot.m_Lift.SetTargetPosition(799.0d, BombasticLift.s_SafetyPower);})
        );


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



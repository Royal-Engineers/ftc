package org.firstinspires.ftc.teamcode.facade.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.BombasticLiftPos;
import org.firstinspires.ftc.teamcode.facade.intake.BombasticLift;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.interfaces.i_gamepad;

@Config
public class controller1 extends i_gamepad {


    private static int LiftIncrement = 30;

    public static double clawAngle1 = 0.81;
    public static double bar1 = -0.06;
    public static double clawAngle2 = 0.82;
    public static double bar2 = -0.07;
    public static double clawAngle3 = 0.81;
    public static double bar3 = -0.07;

    public static double clawAngle4 = 0.82;
    public static double bar4 = -0.08;

    public static double clawAngle5 = 0.84;
    public static double bar5 = -0.099;

    public static double clawAngle6 = 0.87;
    public static double bar6 = -0.115;

    public controller1(Gamepad gamepad, RobotHardware robot){
        super(gamepad, robot);
    }
    @Override
    public void initialize()
    {
        controller.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                new InstantCommand(()->{m_Robot.m_imu.resetYaw();})
        );

        controller.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).toggleWhenPressed(
                new InstantCommand(()->{m_Robot.motorIntake.setPower(0.8f);}),
                new InstantCommand(()->{m_Robot.motorIntake.setPower(0.0f);})
        );

        controller.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(
                new InstantCommand(()->{m_Robot.m_Gyara.setPosition(RobotHardware.s_ClawlClosedPos);}),
                new InstantCommand(()->{m_Robot.m_Gyara.setPosition(RobotHardware.s_ClawOpenPos);})
        );

        controller.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        new BombasticLiftPos(m_Robot, m_Robot.m_Lift, BombasticLift.e_LiftPosition.LowPos),
                        new InstantCommand(()->{m_Robot.m_Gyara.setPosition(RobotHardware.s_ClawOpenPos);}),
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

        controller.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(()->{m_Robot.m_Lift.SetStatePosition(BombasticLift.e_LiftPosition.MidPos);}),
                        new InstantCommand(()->{m_Robot.m_BombaSexy.SetPosition(0.5);}),
                        new InstantCommand(()->{m_Robot.m_GyaraBomba.setPosition(RobotHardware.s_ScoringClawAngle);})
                )
        );

        controller.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(()->{m_Robot.m_Lift.SetStatePosition(BombasticLift.e_LiftPosition.MaxPos);}),
                        new InstantCommand(()->{m_Robot.m_BombaSexy.SetPosition(0.5);}),
                        new InstantCommand(()->{m_Robot.m_GyaraBomba.setPosition(RobotHardware.s_ScoringClawAngle);})
                )        );

        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(()->{m_Robot.m_Gyara.setPosition(RobotHardware.s_ClawOpenPos);}),
                        new WaitCommand(400),
                        new InstantCommand(()->{m_Robot.m_Lift.SetStatePosition(BombasticLift.e_LiftPosition.LowPos);}),
                        new InstantCommand(()->{m_Robot.m_BombaSexy.SetPosition(0.0);}),
                        new InstantCommand(()->{m_Robot.m_GyaraBomba.setPosition(RobotHardware.s_IdleClawAngle);})
                )        );

    }
    double TriggerPos = 0.0f;
    @Override
    public void update() {
        BombasticLift lift = m_Robot.m_Lift;
        TriggerPos = 0.0f;

        if (gamepad.right_trigger > MinPush && lift.GetLiftPosition() < lift.m_MaxPosition)
            TriggerPos = gamepad.right_trigger;
        else if ( gamepad.left_trigger > MinPush && lift.GetLiftPosition() > lift.m_MinPosition )
            TriggerPos = -gamepad.left_trigger;

        if ( Math.abs(TriggerPos) > MinPush ) {
            sg_CommandScheduler.schedule(
                    new InstantCommand(() -> {
                        lift.IncrementTargetPosition(TriggerPos * LiftIncrement);
                    })
            );
        }
        UpdateTelemetry();
    }

    private void UpdateTelemetry()
    {
        if ( RobotHardware.DebugMode == false )
            return;
        m_telemetry.addData("Controller1 Debug -> ", "BEGIN");
        m_telemetry.addData("LeftTrigger: ", gamepad.left_trigger);
        m_telemetry.addData("RightTrigger: ", gamepad.right_trigger);
        m_telemetry.addData("LiftPower: ", TriggerPos);
        m_telemetry.addData("Controller1 Debug -> ", "END\n");
    }

}

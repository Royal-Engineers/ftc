package org.firstinspires.ftc.teamcode.facade.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.BombasticLiftPos;
import org.firstinspires.ftc.teamcode.commands.Transfer;
import org.firstinspires.ftc.teamcode.facade.intake.BombasticLift;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.interfaces.i_gamepad;

@Config
public class controller1 extends i_gamepad {


    private static int LiftIncrement = 30;


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

        controller.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(()->{
                    if ( Math.abs(m_Robot.m_Gyara.getPosition() - RobotHardware.s_ClawlClosedPos) < 0.01)
                        m_Robot.m_Gyara.setPosition(RobotHardware.s_ClawOpenPos);
                    else
                        m_Robot.m_Gyara.setPosition(RobotHardware.s_ClawlClosedPos);

                })
        );

        controller.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                        new Transfer(m_Robot)
        );

        controller.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(()->{m_Robot.m_Lift.SetStatePosition(BombasticLift.e_LiftPosition.MidPos);}),
                        new InstantCommand(()->{m_Robot.m_BombaSexy.SetPosition(0.57);}),
                        new InstantCommand(()->{m_Robot.m_GyaraBomba.setPosition(RobotHardware.s_ScoringClawAngle);})
                )
        );

        controller.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(()->{m_Robot.m_Lift.SetStatePosition(BombasticLift.e_LiftPosition.MaxPos);}),
                        new InstantCommand(()->{m_Robot.m_BombaSexy.SetPosition(0.57);}),
                        new InstantCommand(()->{m_Robot.m_GyaraBomba.setPosition(RobotHardware.s_ScoringClawAngle);})
                )        );

        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(()->{m_Robot.m_Gyara.setPosition(RobotHardware.s_ClawOpenPos);}),
                        new WaitCommand(800),
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

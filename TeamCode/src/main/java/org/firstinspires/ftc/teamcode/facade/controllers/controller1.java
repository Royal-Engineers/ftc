package org.firstinspires.ftc.teamcode.facade.controllers;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.BombasticLiftPos;
import org.firstinspires.ftc.teamcode.facade.intake.BombasticLift;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.interfaces.i_gamepad;

public class controller1 extends i_gamepad {

    private static double MinPush = 0.01d;

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

        controller.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new InstantCommand(()->{m_Robot.m_Lift.SetStatePosition(BombasticLift.e_LiftPosition.MidPos);})
        );

        controller.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new BombasticLiftPos(m_Robot, m_Robot.m_Lift, BombasticLift.e_LiftPosition.MaxPos)
        );

        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new BombasticLiftPos(m_Robot, m_Robot.m_Lift, BombasticLift.e_LiftPosition.LowPos)
        );

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

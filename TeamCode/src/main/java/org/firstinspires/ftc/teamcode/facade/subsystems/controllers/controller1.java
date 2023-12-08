package org.firstinspires.ftc.teamcode.facade.subsystems.controllers;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.GetJPos;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.interfaces.i_gamepad;

public class controller1 extends i_gamepad {
    public controller1(Gamepad gamepad, RobotHardware robot){
        super(gamepad, robot);
    }

    @Override
    public void initialize()
    {
        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(new GetJPos(m_Robot),
                new InstantCommand(()->{GetJPos.Enabled = false;}) );
    }

    @Override
    public void update() {
        m_Robot.m_telemetry.addData("ba", GetJPos.Enabled);
    }
}

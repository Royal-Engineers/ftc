package org.firstinspires.ftc.teamcode.facade.interfaces;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.facade.RobotHardware;

abstract public class i_gamepad {

    protected RobotHardware m_Robot;
    private Gamepad gamepad;
    protected GamepadEx controller;
    private CommandScheduler m_CommandScheduler;

    public i_gamepad(Gamepad gamepad, RobotHardware robot){
        this.gamepad = gamepad;
        this.m_Robot = robot;
        controller = new GamepadEx(gamepad);
        m_CommandScheduler = CommandScheduler.getInstance();

    }

    abstract public void update();
    abstract public void initialize();
}

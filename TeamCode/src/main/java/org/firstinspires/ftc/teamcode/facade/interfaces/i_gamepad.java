package org.firstinspires.ftc.teamcode.facade.interfaces;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;

abstract public class i_gamepad {

    protected RobotHardware m_Robot;
    protected Gamepad gamepad;
    protected GamepadEx controller;

    protected CommandScheduler sg_CommandScheduler;

    protected Telemetry m_telemetry;
    private CommandScheduler m_CommandScheduler;

    protected static double MinPush = 0.01d;


    public i_gamepad(Gamepad gamepad, RobotHardware robot){
        this.gamepad = gamepad;
        this.m_Robot = robot;
        controller = new GamepadEx(gamepad);
        m_CommandScheduler = CommandScheduler.getInstance();
        sg_CommandScheduler = CommandScheduler.getInstance();
        m_telemetry = m_Robot.m_telemetry;

    }

    abstract public void update();
    abstract public void initialize();
}

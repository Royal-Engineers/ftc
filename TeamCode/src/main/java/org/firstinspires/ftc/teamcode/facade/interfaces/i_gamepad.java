package org.firstinspires.ftc.teamcode.facade.interfaces;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;

abstract public class i_gamepad {
    public static double clawAngle1 = 0.81;
    public static double bar1 = -0.066;
    public static double clawAngle2 = 0.82;
    public static double bar2 = -0.07;
    public static double clawAngle3 = 0.81;
    public static double bar3 = -0.07;

    public static double clawAngle4 = 0.82;
    public static double bar4 = -0.08;

    public static double clawAngle5 = 0.84;
    public static double bar5 = -0.099;

    public static double clawAngle6 = 0.87;
    public static double bar6 = -0.14;
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

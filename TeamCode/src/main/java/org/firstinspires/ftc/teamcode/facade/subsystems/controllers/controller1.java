package org.firstinspires.ftc.teamcode.facade.subsystems.controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.interfaces.i_gamepad;

public class controller1 extends i_gamepad {
    public controller1(Gamepad gamepad, RobotHardware robot){
        super(gamepad, robot);
    }

    @Override
    public void update() {

    }
}

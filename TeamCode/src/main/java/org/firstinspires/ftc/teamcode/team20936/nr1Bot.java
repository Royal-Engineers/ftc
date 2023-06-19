package org.firstinspires.ftc.teamcode.team20936;

import com.arcrobotics.ftclib.command.Robot;

public class nr1Bot extends Robot {

    public enum OpMode {
        TELEOP, AUTO
    }

    public nr1Bot (OpMode type) {
        if(type == OpMode.TELEOP) {
            initTele();
        }
        else  {
            initAuto();
        }
    }

    public void initTele() {

    }

    public void initAuto() {

    }

}

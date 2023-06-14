package org.firstinspires.ftc.teamcode.team20936.teleop.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class revSubsystem extends SubsystemBase {

    private final DcMotorEx armRev;
    private final Telemetry telemetry;
    int poz0;

    public revSubsystem(DcMotorEx armRev, int poz0, Telemetry telemetry) {
        this.armRev = armRev;
        this.poz0 = poz0;
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addData("armRevPos ", armRev.getCurrentPosition());
    }

    public void down () {
        armRev.setTargetPosition(-485);
        armRev.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armRev.setPower(0.25);
    }

    public void up () {
        armRev.setTargetPosition(-27);
        armRev.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armRev.setPower(0.5);
    }

    public void setArmRevPos (int target) {
        armRev.setTargetPosition(target);
        armRev.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armRev.setPower(1);
    }


}

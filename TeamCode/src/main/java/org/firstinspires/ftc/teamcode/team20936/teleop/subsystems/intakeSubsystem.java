package org.firstinspires.ftc.teamcode.team20936.teleop.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class intakeSubsystem extends SubsystemBase {

    private final ServoEx claw, wristRev, wrist;
    private final DcMotorEx armRev;
    private final Telemetry telemetry;
    public static double grabPos = 0.00;
    public static double releasePos = 0.22;
    public static double wristFlat = 0.435;
    public static double wristRevFlat = 0.413;
    public static double wristOne= 0.16388888;


    public intakeSubsystem(ServoEx claw, ServoEx wristRev, ServoEx wrist, DcMotorEx armRev, Telemetry telemetry) {
        this.claw = claw;
        this.wristRev = wristRev;
        this.wrist = wrist;
        this.armRev = armRev;
        this.telemetry = telemetry;
    }

    public void periodic() {
        telemetry.addData("claw pos: ", claw.getPosition());
        telemetry.addData("wrist pos: ", wrist.getPosition());
        telemetry.addData("wristRev pos: ", wristRev.getPosition());
        telemetry.addData("armRev pos: ", armRev.getCurrentPosition());
    }

    public void grab() {
        claw.setPosition(grabPos);
    }

    public void release() {
        claw.setPosition(releasePos);
    }

    public void pickUp(){

        wristRev.setPosition(wristRevFlat);
        wrist.setPosition(wristFlat);

        armRev.setTargetPosition(-485);
        armRev.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armRev.setPower(0.3);
    }

    public void lowPos()
    {
        wristRev.setPosition(wristRevFlat);
        wrist.setPosition(wristOne);

        armRev.setTargetPosition(-50);
        armRev.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armRev.setPower(0.5);
    }

    public void setManualWristPos(double target) {
        target*=0.01;
        wrist.setPosition(wrist.getPosition() - target);
    }

    public void setManualWristRevPos(double target) {
        target*=0.01;
        wristRev.setPosition(wristRev.getPosition() - target);
    }

    public void setWristPos(double target) {
        wrist.setPosition(target);
    }

    public void setWristRevPos(double target) {
        wristRev.setPosition(target);
    }

    public void setClawPos(double target) {
        claw.setPosition(target);
    }

    public void setArmRevPos(int target) {
        armRev.setTargetPosition(target);
        armRev.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armRev.setPower(1);
    }


}



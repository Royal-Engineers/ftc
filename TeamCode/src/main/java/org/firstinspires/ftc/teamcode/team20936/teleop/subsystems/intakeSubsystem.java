package org.firstinspires.ftc.teamcode.team20936.teleop.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class intakeSubsystem extends SubsystemBase {

    private final ServoEx claw;
    private final ServoEx wristRev;
    private final ServoEx wrist;
    private final DcMotor armRev;
    private final Telemetry telemetry;
    public static double grabPos = 0.00;
    public static double releasePos = 0.22;
    public static double wristFlat = 0.435;
    public static double wristRevFlat = 0.413;

    public static double wristOne= 0.16388888;
    final int poz0_rev;


    public intakeSubsystem(ServoEx claw, ServoEx wristRev, ServoEx wrist, DcMotor armRev, Telemetry telemetry) {
        this.claw = claw;
        this.wristRev = wristRev;
        this.wrist = wrist;
        this.armRev = armRev; poz0_rev = armRev.getCurrentPosition();
        this.telemetry = telemetry;
    }

    public void periodic() {
        telemetry.addData("claw pos: ", claw.getPosition());
    }

    public void grab() {
        claw.setPosition(grabPos);
    }

    public void release() {
        claw.setPosition(releasePos);
    }

    public void pickUp(){
        armRev.setTargetPosition(poz0_rev-485);
        armRev.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armRev.setPower(0.25);

        wristRev.setPosition(wristRevFlat);
        wrist.setPosition(wristFlat);
    }

    public void lowPos()
    {
        wristRev.setPosition(wristRevFlat);
        wrist.setPosition(wristOne);

        armRev.setTargetPosition(poz0_rev);
        armRev.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armRev.setPower(0.45);
    }

}



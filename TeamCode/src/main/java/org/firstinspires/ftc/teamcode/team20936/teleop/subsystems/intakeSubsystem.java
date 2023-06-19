package org.firstinspires.ftc.teamcode.team20936.teleop.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class intakeSubsystem extends SubsystemBase {

    public final ServoEx claw, wristRev, wrist;
    public final DcMotorEx armRev;
    private final DistanceSensor sensorDistance;
    private final Telemetry telemetry;
    public static double grabPos = 0.00;
    public static double releasePos = 0.22;
    public static double wristFlat = 0.435;
    public static double wristRevFlat = 0.413;
    public static double wristOne= 0.16388888;
    private boolean toggleStack = false;
    private boolean sens_toggle = true;


    public intakeSubsystem(HardwareMap hMap, Telemetry telemetry) {

        claw = new SimpleServo(hMap, "servo_gheara", 0, 1);
        claw.setPosition(Range.clip(0.22, 0, 1));

        wristRev = new SimpleServo(hMap, "servo_brat3", 0, 1);
        wristRev.setPosition(Range.clip(0.4, 0, 1));

        wrist = new SimpleServo(hMap, "servo_brat2", 0, 1);
        wrist.setPosition(Range.clip(0.37, 0, 1));

        armRev = hMap.get(DcMotorEx.class, "rev_hd_brat");
        armRev.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armRev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sensorDistance = hMap.get(DistanceSensor.class, "senzor_gheara");

        this.telemetry = telemetry;

    }

    public void periodic() {
        telemetry.addData("claw pos: ", claw.getPosition());
        telemetry.addData("wrist pos: ", wrist.getPosition());
        telemetry.addData("wristRev pos: ", wristRev.getPosition());
        telemetry.addData("armRev pos: ", armRev.getCurrentPosition());
        telemetry.addData("sensor active: ", sens_toggle);
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

    public void stack_5thcone () {
        claw.setPosition(releasePos);

        armRev.setTargetPosition(-0); // de gasit pozitia
        armRev.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armRev.setPower(0.25);

        wrist.setPosition(wristFlat); // de gasit pozitia
        wristRev.setPosition(wristRevFlat); // de gasit pozitia
    }

    public void stack_4thcone() {
        claw.setPosition(releasePos);

        armRev.setTargetPosition(-0); // de gasit pozitia
        armRev.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armRev.setPower(0.25);

        wrist.setPosition(wristFlat); // de gasit pozitia
        wristRev.setPosition(wristRevFlat); // de gasit pozitia
    }

    public void stack_3rdcone () {
        claw.setPosition(releasePos);

        armRev.setTargetPosition(-0); // de gasit pozitia
        armRev.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armRev.setPower(0.25);

        wrist.setPosition(wristFlat); // de gasit pozitia
        wristRev.setPosition(wristRevFlat); // de gasit pozitia
    }

    public void stack_2ndcone () {
        claw.setPosition(releasePos);

        armRev.setTargetPosition(-0); // de gasit pozitia
        armRev.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armRev.setPower(0.25);

        wrist.setPosition(wristFlat); // de gasit pozitia
        wristRev.setPosition(wristRevFlat); // de gasit pozitia
    }

    public void setToggleStack (boolean value) { toggleStack = value; }

    public boolean getToggleStack() { return toggleStack; }

    public void setManualRevPos(double input) {
        input*= 10;

        armRev.setTargetPosition(armRev.getCurrentPosition() - (int)input);
        armRev.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armRev.setPower(0.5);
    }

    public boolean coneDetected () {
        if(sensorDistance.getDistance(DistanceUnit.MM) < 30) {
            return true;
        }
        else {
            return false;
        }
    }

    public void toggleSensor() { sens_toggle = !sens_toggle; }

    public boolean getSensorActive() { return sens_toggle; }


}



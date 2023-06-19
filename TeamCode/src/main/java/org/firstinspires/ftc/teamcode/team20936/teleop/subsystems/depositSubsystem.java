package org.firstinspires.ftc.teamcode.team20936.teleop.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class depositSubsystem extends SubsystemBase {

    private final DcMotorEx leftMotor, rightMotor;
    private final ServoEx latch;
    private final Telemetry telemetry;
    private static int middlePos = 1010, highPos = 1720;
    double leftPower, rightPower;
    private int height = 1;

    public depositSubsystem (HardwareMap hMap, Telemetry telemetry){

        leftMotor = hMap.get(DcMotorEx.class, "liftStanga");
        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor = hMap.get(DcMotorEx.class, "liftDreapta");
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        latch= new SimpleServo(hMap, "servo_deposit", 0, 1);
        latch.setPosition(Range.clip(0.1, 0, 1));

        this.telemetry = telemetry;

    }

    @Override
    public void periodic() {
        telemetry.addData("slideLeftPos: ", leftMotor.getCurrentPosition());
        telemetry.addData("slideRightPos: ", rightMotor.getCurrentPosition());
        telemetry.addData("latchPos: ", latch.getPosition());
    }

    public void extend () {
        double currentPos = -leftMotor.getCurrentPosition();
        int targetPos;
        if(height == 1) { targetPos = middlePos; }
        else { targetPos = highPos; }

        if (currentPos < targetPos) {
            leftPower = -1;
            rightPower = 1;
        } else if (currentPos > targetPos) {
            leftPower = 1;
            rightPower = -1;
        }

        leftMotor.setTargetPosition((int) -targetPos);
        rightMotor.setTargetPosition((int) targetPos);

        leftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    public void retract () {
        double currentPos = -leftMotor.getCurrentPosition();

        if (currentPos < 0) {
            leftPower = -1;
            rightPower = 1;
        } else if (currentPos > 0) {
            leftPower = 1;
            rightPower = -1;
        }

        leftMotor.setTargetPosition((int) -0);
        rightMotor.setTargetPosition((int) 0);

        leftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    public void latchOpen () {
        latch.setPosition(0.1);
    }
    public void latchClose () {
        latch.setPosition(0.23);
    }

    public void manualControl (double target) {
        double currentPos = -leftMotor.getCurrentPosition();

        if (currentPos < target) {
            leftPower = -1;
            rightPower = 1;
        } else if (currentPos > target) {
            leftPower = 1;
            rightPower = -1;
        }
        leftMotor.setTargetPosition((int) -target);
        rightMotor.setTargetPosition((int) target);

        leftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }


    public int getPosition() {
        return (-leftMotor.getCurrentPosition())/* - (right.getCurrentPosition())) / 2 */;
    }

    public void setHeight(int height) { this.height = height; }

}

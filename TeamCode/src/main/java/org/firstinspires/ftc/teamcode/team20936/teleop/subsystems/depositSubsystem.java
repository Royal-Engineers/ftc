package org.firstinspires.ftc.teamcode.team20936.teleop.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class depositSubsystem extends SubsystemBase {

    private final DcMotorEx leftMotor, rightMotor;
    private final ServoEx latch;
    private final Telemetry telemetry;
    private static int middlePos = 1010, highPos = 1720;
    double leftPower, rightPower;

    public depositSubsystem (DcMotorEx leftMotor, DcMotorEx rightMotor, ServoEx latch, Telemetry telemetry){
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.latch = latch;
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addData("slideLeftPos: ", leftMotor.getCurrentPosition());
        telemetry.addData("slideRightPos: ", rightMotor.getCurrentPosition());
        telemetry.addData("latchPos: ", latch.getPosition());
    }

    public void raiseMiddle () {
        double currentPos = -leftMotor.getCurrentPosition();

        if (currentPos < middlePos) {
            leftPower = -1;
            rightPower = 1;
        } else if (currentPos > middlePos) {
            leftPower = 1;
            rightPower = -1;
        }

        leftMotor.setTargetPosition((int) -middlePos);
        rightMotor.setTargetPosition((int) middlePos);

        leftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    public void raiseHigh () {
        double currentPos = -leftMotor.getCurrentPosition();

        if (currentPos < highPos) {
            leftPower = -1;
            rightPower = 1;
        } else if (currentPos > highPos) {
            leftPower = 1;
            rightPower = -1;
        }

        leftMotor.setTargetPosition((int) -highPos);
        rightMotor.setTargetPosition((int) highPos);

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

}

package org.firstinspires.ftc.teamcode.team20936.teleop.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class depositSubsystem extends SubsystemBase {

    private final DcMotorEx leftMotor, rightMotor;
    private final ServoEx latch;
    private final Telemetry telemetry;
    private static int middlePos = 299, highPos = 520;
    double leftPower, rightPower;

    double targetPosition = 0;
    public depositSubsystem (DcMotorEx leftMotor, DcMotorEx rightMotor, ServoEx latch, Telemetry telemetry){
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.latch = latch;
        this.telemetry = telemetry;
        targetPosition = 0;
    }

    public static double P = 0.01, I = 0.0, D = 0.01;
    public PIDController PIDlift = new PIDController(P, I, D);

    double currentPos;

    @Override
    public void periodic() {
        telemetry.addData("slideLeftPos: ", leftMotor.getCurrentPosition());
        telemetry.addData("slideRightPos: ", rightMotor.getCurrentPosition());
        telemetry.addData("latchPos: ", latch.getPosition());

        currentPos = -leftMotor.getCurrentPosition();
       double output = ( PIDlift.calculate(currentPos, targetPosition));

        telemetry.addData("Output: ", output);
        telemetry.addData("Pos: ", currentPos);

        telemetry.addData("TargetPos: ", targetPosition);



    }

    public void raiseMiddle () {
        targetPosition = middlePos;
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
        targetPosition = highPos;
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
        targetPosition = 0;
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

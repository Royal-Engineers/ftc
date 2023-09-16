package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class test extends LinearOpMode {

    AnalogInput encoder;
    CRServo servo;
    DcMotor motor;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class,"motorFrontRight");
        servo = hardwareMap.get(CRServo.class, "servoFrontRight");
        encoder = hardwareMap.get(AnalogInput.class, "encoderFrontRight");


        waitForStart();
while(opModeIsActive()) {
    servo.setPower(gamepad1.left_stick_x);
    telemetry.addData("OX1:", gamepad1.left_stick_x);
    telemetry.addData("OY1:", gamepad1.left_stick_y);
    telemetry.addData("OX2:", gamepad1.right_stick_x);
    telemetry.addData("OY2:",gamepad1.right_stick_y);
    telemetry.update();
}

    }

}

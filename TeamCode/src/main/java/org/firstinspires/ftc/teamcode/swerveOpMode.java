package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class swerveOpMode extends LinearOpMode {

    DcMotorEx motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft;
    CRServo servoFrontRight, servoFrontLeft, servoBackRight, servoBackLeft;
    AnalogInput aencoderFrontRight, aencoderFrontLeft, aencoderBackRight, aencoderBackLeft;
    AbsoluteAnalogEncoder encoderFrontRight, encoderFrontLeft, encoderBackLeft, encoderBackRight;
    GamepadEx controller1;

    double L = 32, W=25;
    double R = Math.hypot(L/2, W/2); // length, width to be updated

    swerveModule moduleFrontRight, moduleFrontLeft, moduleBackLeft, moduleBackRight;

    @Override
    public void runOpMode() {


        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        servoFrontRight = hardwareMap.get(CRServo.class, "servoFrontRight");
        servoFrontLeft = hardwareMap.get(CRServo.class, "servoFrontLeft");
        servoBackRight = hardwareMap.get(CRServo.class, "servoBackRight");
        servoBackLeft = hardwareMap.get(CRServo.class, "servoBackRight");

        aencoderFrontRight = hardwareMap.get(AnalogInput.class, "encoderFrontRight");
        aencoderFrontLeft = hardwareMap.get(AnalogInput.class, "encoderFrontLeft");
        aencoderBackRight = hardwareMap.get(AnalogInput.class, "encoderBackRight");
        aencoderBackLeft = hardwareMap.get(AnalogInput.class, "encoderBackLeft");
        encoderFrontRight = new AbsoluteAnalogEncoder(aencoderFrontRight, 53, true);
        encoderFrontLeft = new AbsoluteAnalogEncoder(aencoderFrontLeft, 0, true);
        encoderBackLeft = new AbsoluteAnalogEncoder(aencoderBackLeft, 0, true);
        encoderBackRight = new AbsoluteAnalogEncoder(aencoderBackRight, 314, true);

        moduleFrontRight = new swerveModule(motorFrontRight, servoFrontRight, encoderFrontRight, telemetry);
        moduleFrontLeft = new swerveModule(motorFrontLeft, servoFrontLeft, encoderFrontLeft, telemetry);
        moduleBackLeft = new swerveModule(motorBackLeft, servoBackLeft, encoderBackLeft, telemetry);
        moduleBackRight = new swerveModule(motorBackRight, servoBackRight, encoderBackRight, telemetry);


        controller1 = new GamepadEx(gamepad1);

        waitForStart();
        while(opModeIsActive()) {

            double FWD = -gamepad1.left_stick_y;
            double STR = gamepad1.left_stick_x;
            double RCW = gamepad1.right_stick_x;

            double A = STR - RCW*(L/R);
            double B = STR + RCW*(L/R);
            double C = FWD - RCW*(W/R);
            double D = FWD + RCW*(W/R);

            double ws1 = Math.hypot(B, C); double wa1 = Math.atan2(B, C)*180/Math.PI;
            double ws2 = Math.hypot(B, D); double wa2 = Math.atan2(B, D)*180/Math.PI;
            double ws3 = Math.hypot(A, D); double wa3 = Math.atan2(A, D)*180/Math.PI;
            double ws4 = Math.hypot(A, C); double wa4 = Math.atan2(A, C)*180/Math.PI;

            double max = ws1;
            if(ws2 > max) max = ws2; if(ws3 > max) max = ws3; if(ws4 > max) max = ws4;
            if(max > 1) { ws1/=max; ws2/=max; ws3/=max; ws4/=max; }

            telemetry.addData("OX1:", gamepad1.left_stick_x);
            telemetry.addData("OY1:", gamepad1.left_stick_y);
            telemetry.addData("OX2:", gamepad1.right_stick_x);
            telemetry.addData("OY2:",gamepad1.right_stick_y);
            telemetry.addData("ws1: ", ws1);
            telemetry.addData("wa1: ", wa1);

            /*telemetry.addData("ws4: ", ws4);
            telemetry.addData("wa4: ", wa4);
            telemetry.addData("ws2: ", ws2);
            telemetry.addData("wa2: ", wa2);


            telemetry.addData("ws4: ", ws4);
            telemetry.addData("wa4: ", wa4);*/

            //telemetry.addData("\n", " ");

            //telemetry.addData("Actual angle frontRight ", encoderFrontRight.getVoltage() / 3.3 * 360);
            //telemetry.addData("Actual angle backRight ", encoderBackRight.getVoltage() / 3.3 * 360);

            telemetry.update();

            /*moduleFrontRight.drive(ws1, wa1);
            moduleBackRight.drive(ws4, wa4);
            moduleFrontLeft.drive(ws2, wa2);

            moduleBackRight.drive(ws4, wa4);*/


        }

    }

}

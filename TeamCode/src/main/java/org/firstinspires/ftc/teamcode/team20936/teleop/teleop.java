package org.firstinspires.ftc.teamcode.team20936.teleop;

import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.Constants;
import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.Lift;

@TeleOp
public class teleop extends LinearOpMode{

    int lift_height = 0;
    boolean gheara_toggle = true;
    boolean brat_rev_toggle = true;
    boolean brat_sus_toggle = true;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Servo servo_brat2 = hardwareMap.servo.get("servo_brat2"); servo_brat2.setPosition(Range.clip(0.37, 0, 1));
        Servo servo_brat3 = hardwareMap.servo.get("servo_brat3"); servo_brat3.setPosition(Range.clip(0.5, 0, 1));
        Servo servo_gheara = hardwareMap.servo.get("servo_gheara"); servo_gheara.setPosition(Range.clip(0.3, 0, 1));

        Servo servo_deposit = hardwareMap.servo.get("servo_deposit"); servo_deposit.setPosition(Range.clip(0, 0, 1));

        DistanceSensor sensorDistanta_intake = hardwareMap.get(DistanceSensor.class, "senzor_gheara");

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft"); motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft"); motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight"); motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight"); motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor rev_hd_brat = hardwareMap.dcMotor.get("rev_hd_brat"); rev_hd_brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rev_hd_brat.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Lift lift = new Lift();
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.init(hardwareMap);
        waitForStart();


        while (opModeIsActive())
        {

            if (gamepad2.a) {
                if (brat_rev_toggle) {
                    rev_hd_brat.setTargetPosition(-420);
                    rev_hd_brat.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rev_hd_brat.setPower(0.1);
                    brat_rev_toggle = !brat_rev_toggle;
                }
                else {
                    rev_hd_brat.setTargetPosition(2);
                    rev_hd_brat.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rev_hd_brat.setPower(-0.5);
                    brat_rev_toggle = !brat_rev_toggle;
                }
            }
            if (gamepad1.options) {
                if (brat_sus_toggle) {
                    servo_deposit.setPosition(Range.clip(0, 0, 1));
                    brat_sus_toggle = !brat_sus_toggle;
                }
                else {
                    servo_deposit.setPosition(Range.clip(0.23, 0, 1));
                    brat_sus_toggle = !brat_sus_toggle;
                }
            }

            telemetry.addData("Status", "Running");

            if (gamepad1.a) {
                lift_height = 1;
            } else if (gamepad1.b) {
                lift_height = 2;
            } else if (gamepad1.y) {
                lift_height = 3;
            } else if (gamepad1.right_trigger > 0.01 && lift.getPosition()<2650)
                    lift.moveLift(lift.getPosition() + 80 * (Math.abs((int) gamepad1.right_trigger)));
             else if (gamepad1.left_trigger > 0.01) {
                if (lift.getPosition() > 0)
                    lift.moveLift(lift.getPosition() - 80 * (Math.abs((int) gamepad1.left_trigger)));
            }

            if (gamepad1.left_bumper)
                switch (lift_height) {
                    case 1: {
                        lift.moveLift(Constants.LiftTargets.LOW);
                        break;
                    }
                    case 2: {
                        lift.moveLift(Constants.LiftTargets.MEDIUM);
                        break;
                    }
                    case 3: {
                        lift.moveLift(Constants.LiftTargets.HIGH);
                        break;
                    }
                }
            else if (gamepad1.right_bumper)
                lift.moveLift(1);

            if(servo_gheara.getPosition() > 0.2 && sensorDistanta_intake.getDistance(DistanceUnit.MM) < 30)
            {
                    gheara_toggle = !gheara_toggle;
                    servo_gheara.setPosition(Range.clip(0.0, 0, 1));
                    sleep(200);
                    servo_brat2.setPosition(Range.clip(0.26,0 , 1));
                    servo_brat3.setPosition(Range.clip(0.5, 0, 1));

                    TimeUnit.MILLISECONDS.sleep(250);

                    gamepad2.rumble(200);
                    telemetry.addData("auto pickup", "picked");
            }
            if (gamepad2.right_bumper)
            {

                gheara_toggle = !gheara_toggle;
                if (gheara_toggle) { servo_gheara.setPosition(Range.clip(0.3, 0, 1)); }
                else { servo_gheara.setPosition(Range.clip(0.0, 0, 1)); TimeUnit.MILLISECONDS.sleep(400);} // DESCHIS ( SE EXECUTA PRIMA)

                TimeUnit.MILLISECONDS.sleep(250);

            }

            if(gamepad2.right_trigger > 0.3)
            {
                servo_brat2.setPosition(Range.clip(0.37,0,1));
                servo_brat3.setPosition(Range.clip(0.5, 0, 1));
            }

            if(gamepad2.right_stick_y > 0.01 && servo_brat2.getPosition() >= 0)
            {
                double dist = Math.abs(gamepad2.right_stick_y) * 0.01;

                servo_brat2.setPosition(Range.clip(servo_brat2.getPosition() - dist, 0, 1)) ;
            }
            else if(gamepad2.right_stick_y < 0.01 && servo_brat2.getPosition() <= 0.7)
            {
                double dist = Math.abs(gamepad2.right_stick_y) * 0.01;

                servo_brat2.setPosition(Range.clip(servo_brat2.getPosition() + dist, 0, 1));
            }




            if(gamepad2.right_stick_x > 0.01 && servo_brat3.getPosition() >= 0)
            {
                double dist = Math.abs(gamepad2.right_stick_x) * 0.01;

                servo_brat3.setPosition(Range.clip(servo_brat3.getPosition() - dist, 0, 1)) ;
            }
            else if(gamepad2.right_stick_x < 0.01 && servo_brat3.getPosition() <= 1)
            {
                double dist = Math.abs(gamepad2.right_stick_x) * 0.01;

                servo_brat3.setPosition(Range.clip(servo_brat3.getPosition() + dist, 0, 1));
            }



            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            motorFrontLeft.setPower(r * Math.cos(robotAngle) + rightX);
            motorFrontRight.setPower(r * Math.sin(robotAngle) - rightX);
            motorBackLeft.setPower(r * Math.sin(robotAngle) + rightX);
            motorBackRight.setPower(r * Math.cos(robotAngle) - rightX);

            telemetry.addData("poz lift", lift.getPosition());
            telemetry.addData("rev poz",  rev_hd_brat.getCurrentPosition());
            telemetry.addData("servo_brat2 pos: ", servo_brat2.getPosition());
            telemetry.addData("servo_brat3 pos: ", servo_brat3.getPosition());
            telemetry.update();
        }

    }

}

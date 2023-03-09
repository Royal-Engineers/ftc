package org.firstinspires.ftc.teamcode.team20936.teleop;

import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.Constants;
import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.Lift;

@TeleOp
public class ProiectPrincipal extends LinearOpMode {
    // private DcMotor motorFrontLeft = null;
    // private DcMotor motorFrontRight = null;
    // private DcMotor motorBackLeft = null;
    // private DcMotor motorBackRight = null;

    // double poz_servo1=0.5;
    double MIN_POSITION = 0;
    double MAX_POSITION = 1;
    boolean gheara_toggle = true;
    boolean am_con = false;
    // long t; // gheara
    // boolean okg=true; // pot intra sa verific daca e apasat?

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // --- lift init, vezi ../subsystems/Lift.java
        Lift lift = new Lift();
        // ---

        // --- motoare sasiu init
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        // ---

        // --- servo init
        Servo servo_gheara = hardwareMap.servo.get("servo_gheara");
        // CRServo servo1 = hardwareMap.crservo.get("servoRotatie");
        // CRServo servo2 = hardwareMap.crservo.get("gheara1");
        // CRServo servo3 = hardwareMap.crservo.get("gheara2");

        // ---

        // --- motoare sasiu directie, pe zero input/power = stationar

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // ---

        waitForStart();

        lift.init(hardwareMap);

        if (isStopRequested())
            return;

        while (opModeIsActive()) {
            // --- Moving lift
            if (gamepad1.dpad_up) {
                lift.moveLift(Constants.LiftTargets.HIGH);
            } else if (gamepad1.dpad_right) {
                lift.moveLift(Constants.LiftTargets.LOW);
            } else if (gamepad1.dpad_left) {
                lift.moveLift(Constants.LiftTargets.MEDIUM);
            } else if (gamepad1.dpad_down) {
                // belt.moveBelt(Constants.IntakeTargets.UP);
                lift.moveLift(Constants.LiftTargets.PICKUP);
            } else if (gamepad1.right_trigger > 0.01) {
                if (lift.getPosition() < 3950)
                    lift.moveLift(lift.getPosition() + (150 * Math.abs((int) gamepad1.right_trigger)));
            } else if (gamepad1.left_trigger > 0.01) {
                if (lift.getPosition() > 150)
                    lift.moveLift(lift.getPosition() - (150 * Math.abs((int) gamepad1.left_trigger)));
                else if (lift.getPosition() > 20)
                    lift.moveLift(lift.getPosition() - 20); // aici e plus fiindca LSY devine negativ
                else if (lift.getPosition() > 2)
                    lift.moveLift(lift.getPosition() - 2); // control mai bun
            }

            // ---

            // --- servo pozitii

            if (gamepad1.a) {
                gheara_toggle = !gheara_toggle;
                if (gheara_toggle) {
                    servo_gheara.setPosition(Range.clip(0.06, MIN_POSITION, MAX_POSITION));
                    // TimeUnit.MILLISECONDS.sleep(30);
                } else {
                    servo_gheara.setPosition(Range.clip(0.13, MIN_POSITION, MAX_POSITION));
                    // TimeUnit.MILLISECONDS.sleep(30);
                }
                TimeUnit.MILLISECONDS.sleep(250);
            }
            telemetry.addData("gheara pozitie", servo_gheara.getPosition());
            // ---

            // --- gamepad 1 directie

            // motorFrontLeft.setPower(frontLeftPower);
            // motorBackLeft.setPower(backLeftPower);
            // motorFrontRight.setPower(frontRightPower);
            // motorBackRight.setPower(backRightPower);

            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            motorFrontLeft.setPower(v1);
            motorFrontRight.setPower(v2);
            motorBackLeft.setPower(v3);
            motorBackRight.setPower(v4);

            // ---

            // uncomment:
            // telemetry.addData("Mode", "running");
            // telemetry.addData("servo1 power", " = " + servo1.getPower());
            // telemetry.addData("servo2 power", " = " + servo2.getPower());
            // telemetry.addData("servo3 power", " = " + servo3.getPower());
            // telemetry.addData("Lift Position", lift.getPosition());
            // telemetry.addData("A2", gamepad2.a); // gheara
            // // -- lift
            // telemetry.addData("Dpad2 Up", gamepad2.dpad_up);
            // telemetry.addData("Dpad2 right", gamepad2.dpad_right);
            // telemetry.addData("Dpad2 down", gamepad2.dpad_down);
            // telemetry.addData("Dpad2 left", gamepad2.dpad_left);
            // // --
            // telemetry.update();
        }
    }
}
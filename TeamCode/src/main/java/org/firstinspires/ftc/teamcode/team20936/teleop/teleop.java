package org.firstinspires.ftc.teamcode.team20936.teleop;

import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.Constants;
import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.Lift;

@TeleOp
public class teleop extends LinearOpMode{

    int val_de_dat_la_lift = 0;

    double MIN_POSITION = 0;
    double MAX_POSITION = 1;
    boolean gheara_toggle = true;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Lift lift = new Lift();

        PIDFController pidf = new PIDFController(0.37, 0.05, 1.02, 0.7);

        GamepadEx controller1 = new GamepadEx(gamepad1);

        Servo servo_gheara = hardwareMap.servo.get("servo_gheara");
        Servo servo_brat1 = hardwareMap.servo.get("servo_brat1");
        Servo servo_brat2 = hardwareMap.servo.get("servo_brat2");
        DistanceSensor sensorDistanta = hardwareMap.get(DistanceSensor.class, "senzor_gheara");

        Motor motorFrontRight = new Motor(hardwareMap, "motorFrontRight", Motor.GoBILDA.RPM_312);
        Motor motorFrontLeft = new Motor(hardwareMap, "motorFrontLeft", Motor.GoBILDA.RPM_312);
        Motor motorBackRight = new Motor(hardwareMap, "motorBackRight", Motor.GoBILDA.RPM_312);
        Motor motorBackLeft = new Motor(hardwareMap, "motorBackLeft", Motor.GoBILDA.RPM_312);

        motorFrontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        MecanumDrive mecanum = new MecanumDrive (motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);

        servo_brat1.setPosition(Range.clip(0.55, MIN_POSITION, MAX_POSITION));
        servo_brat2.setPosition(Range.clip(0.5, MIN_POSITION, MAX_POSITION));
        servo_gheara.setPosition(Range.clip(0.3, MIN_POSITION, MAX_POSITION));

        waitForStart();

        lift.init(hardwareMap);

        while (opModeIsActive())
        {
            // --- Moving lift
            if (gamepad1.a) {
                val_de_dat_la_lift = 1;
//                lift.moveLift(Constants.LiftTargets.HIGH);
            } else if (gamepad1.b) {
                val_de_dat_la_lift = 2;
//                lift.moveLift(Constants.LiftTargets.LOW);
            } else if (gamepad1.y) {
                val_de_dat_la_lift = 3;
//                lift.moveLift(Constants.LiftTargets.MEDIUM);
//            } else if (gamepad1.dpad_down) {
                // belt.moveBelt(Constants.IntakeTargets.UP);
//                lift.moveLift(Constants.LiftTargets.PICKUP);
            } else if (gamepad1.right_trigger > 0.01 && lift.getPosition()<2650)
                    lift.moveLift(lift.getPosition() + 80 * (Math.abs((int) gamepad1.right_trigger)));
             else if (gamepad1.left_trigger > 0.01) {
                if (lift.getPosition() > 0)
                    lift.moveLift(lift.getPosition() - 80 * (Math.abs((int) gamepad1.left_trigger)));
            }

            if (gamepad1.left_bumper)
                switch (val_de_dat_la_lift) {
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

            telemetry.addData("Status", "Running");

            mecanum.driveRobotCentric(controller1.getRightX(),
                    -controller1.getLeftY(),
                    controller1.getLeftX());

            if(servo_gheara.getPosition() < 0.3)
            {
                if(sensorDistanta.getDistance(DistanceUnit.MM) < 50)
                {
                    gheara_toggle = !gheara_toggle;
                    servo_gheara.setPosition(Range.clip(0.0, MIN_POSITION, MAX_POSITION));

                    TimeUnit.MILLISECONDS.sleep(250);

                    gamepad1.rumble(200);
                    telemetry.addData("auto pickup", "picked");
                }
            }


            if (gamepad1.x)
            {

                gheara_toggle = !gheara_toggle;
                if (gheara_toggle) { servo_gheara.setPosition(Range.clip(0.3, MIN_POSITION, MAX_POSITION)); }
                else { servo_gheara.setPosition(Range.clip(0.0, MIN_POSITION, MAX_POSITION)); TimeUnit.MILLISECONDS.sleep(400);} // DESCHIS ( SE EXECUTA PRIMA)

                TimeUnit.MILLISECONDS.sleep(250);


            }
            telemetry.addData("lift pos", lift.getPosition());
            telemetry.update();
        }

    }

}

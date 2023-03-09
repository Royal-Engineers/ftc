package org.firstinspires.ftc.teamcode.team20936.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.controller.PIDFController;

@TeleOp
public class controller extends LinearOpMode{



    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        PIDFController pidf = new PIDFController(0.37, 0.05, 1.02, 0.7);

        GamepadEx controller1 = new GamepadEx(gamepad1);

        Motor motorFrontRight = new Motor(hardwareMap, "motorFrontRight", Motor.GoBILDA.RPM_312);
        Motor motorFrontLeft = new Motor(hardwareMap, "motorFrontLeft", Motor.GoBILDA.RPM_312);
        Motor motorBackRight = new Motor(hardwareMap, "motorBackRight", Motor.GoBILDA.RPM_312);
        Motor motorBackLeft = new Motor(hardwareMap, "motorBackLeft", Motor.GoBILDA.RPM_312);

        motorFrontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        MecanumDrive mecanum = new MecanumDrive (motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);

        waitForStart();

        while (opModeIsActive())
        {

            mecanum.driveRobotCentric(-controller1.getRightX(),
                    -controller1.getLeftY(),
                    -controller1.getLeftX());

            telemetry.addData("Status", "Running");
            telemetry.update();
        }

    }

}

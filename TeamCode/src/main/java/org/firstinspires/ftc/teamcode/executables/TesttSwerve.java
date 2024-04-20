package org.firstinspires.ftc.teamcode.executables;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class TesttSwerve extends CommandOpMode {
    public DcMotorEx motorFrontRight;
    @Override
    public void initialize()
    {
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorFrontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void run()
    {
        motorFrontRight.setPower(1);
        telemetry.addData("miliamps front right: ", motorFrontRight.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.update();
    }
}

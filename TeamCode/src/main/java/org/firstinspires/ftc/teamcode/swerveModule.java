package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class swerveModule {

    DcMotorEx motor;
    CRServo servo;
    AbsoluteAnalogEncoder encoder;

    public static double target = 0;
    boolean motorDirection = true;
    public static double P = 0.01 , I = 0, D = 0.0, F = 0;

    public static double tolerance = 2;
    PIDFController angleController = new PIDFController(P, I, D, F);
   Telemetry telemetrieba;
    public swerveModule(DcMotorEx motor, CRServo servo, AbsoluteAnalogEncoder encoder, Telemetry telemetry) {
        this.motor = motor; this.servo = servo; this.encoder = encoder;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
telemetrieba = telemetry;
        angleController.setTolerance(tolerance);
    }

    public void drive(double motorPower, double servoAngle) {

        target = servoAngle + 180;
        target = 360 - target;

        double current = encoder.getCurrentPosition();
        double error;

        double errorpoz, errorneg;

        if ( current >= target){
            errorpoz = (360 - current) + target;
            errorneg = current - target;
        }
        else{
            errorpoz = target - current;
            errorneg = current + ( 360 - target );
        }
        errorpoz *= - 1;

        if ( Math.abs(errorpoz) < Math.abs(errorneg) )
            error = errorpoz;
        else error = errorneg;

/*
        error =target - current;
*/
        /*normalizeDegrees(error);*/

       double power = angleController.calculate(0, error);
       if(Double.isNaN(power)) power = 0;
        if ( Math.abs(error) < tolerance )
            power = 0;

       servo.setPower(power);


       telemetrieba.addData("Pow: ", power);
        telemetrieba.addData("PowMot: ", motorPower);
       telemetrieba.addData("Target ", target);
       telemetrieba.addData("Pos ", current);
       telemetrieba.addData("Eroor: ", error);
       telemetrieba.addData("EroorPoz: ", errorpoz);
       telemetrieba.addData("EroorNeg: ", errorneg);




        motor.setPower(motorPower);

    }


}

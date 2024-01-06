package org.firstinspires.ftc.teamcode.facade.drive;

import static java.lang.Math.atan2;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;


public class DriveSubsystem {

    public boolean TelemeteryEnabled = true;
    private RobotHardware robot;
    private double L = 27, W=25;
    private double R = Math.hypot(L/2, W/2);
    private swerveModule moduleFrontRight, moduleFrontLeft, moduleBackLeft, moduleBackRight;
    private Gamepad gamepad1;
    private double ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4;
    IMU imu;

    public DriveSubsystem(RobotHardware robot){
        this.robot = robot;
        moduleBackRight = robot.moduleBackRight;
        moduleBackLeft = robot.moduleBackLeft;

        moduleFrontRight = robot.moduleFrontRight;
        moduleFrontLeft = robot.moduleFrontLeft;

        gamepad1 = robot.m_gamepad1;

        imu = this.robot.m_imu;
        imu.initialize(this.robot.imu_parameters);
        imu.resetYaw();

    }
    public void UpdateGamepad(){

        //chinematici
        double FWD = -gamepad1.left_stick_y;
        double STR = gamepad1.left_stick_x;
        double RCW = gamepad1.right_stick_x;

        double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double temp = FWD*Math.cos(botHeading) + STR*Math.sin(botHeading);
        STR = -FWD*Math.sin(botHeading) + STR*Math.cos(botHeading);
        FWD = temp;

        // ***UNCOMMENT FOR SUSSY BAKA FIELD CENTRIC*****

        double A = STR - RCW*(L/R);
        double B = STR + RCW*(L/R);
        double C = FWD - RCW*(W/R);
        double D = FWD + RCW*(W/R);

        ws1 = Math.hypot(B, C);  wa1 = Math.atan2(B, C)*180/Math.PI;
        ws2 = Math.hypot(B, D);  wa2 = Math.atan2(B, D)*180/Math.PI;
        ws3 = Math.hypot(A, D);  wa3 = Math.atan2(A, D)*180/Math.PI;
        ws4 = Math.hypot(A, C);  wa4 = Math.atan2(A, C)*180/Math.PI;


        //normalizare
        double max = ws1;
        if(ws2 > max) max = ws2; if(ws3 > max) max = ws3; if(ws4 > max) max = ws4;
        if(max > 1) { ws1/=max; ws2/=max; ws3/=max; ws4/=max; }
        update();
    }

    public void UpdateAuto(double fwd, double str, double rcw){

        //chinematici
        double FWD = fwd;
        double STR = str;
        double RCW = rcw;

        double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double temp = FWD*Math.cos(botHeading) + STR*Math.sin(botHeading);
        STR = -FWD*Math.sin(botHeading) + STR*Math.cos(botHeading);
        FWD = temp;

        // ***UNCOMMENT FOR SUSSY BAKA FIELD CENTRIC*****

        double A = STR - RCW*(L/R);
        double B = STR + RCW*(L/R);
        double C = FWD - RCW*(W/R);
        double D = FWD + RCW*(W/R);

        ws1 = Math.hypot(B, C);  wa1 = atan2(B, C)*180/Math.PI;
        ws2 = Math.hypot(B, D);  wa2 = atan2(B, D)*180/Math.PI;
        ws3 = Math.hypot(A, D);  wa3 = atan2(A, D)*180/Math.PI;
        ws4 = Math.hypot(A, C);  wa4 = atan2(A, C)*180/Math.PI;


        //normalizare
        double max = ws1;
        if(ws2 > max) max = ws2; if(ws3 > max) max = ws3; if(ws4 > max) max = ws4;
        if(max > 1) { ws1/=max; ws2/=max; ws3/=max; ws4/=max; }

        update();

    }
    private void update(){
        drive();
        AddTelemetry();
    }

    private void drive(){
        moduleFrontRight.drive(ws1, wa1);
        moduleFrontLeft.drive(ws2, wa2);
        moduleBackLeft.drive(ws3, wa3);
        moduleBackRight.drive(ws4, wa4);
    }

    private void AddTelemetry(){
        if ( !TelemeteryEnabled )
            return;

        robot.m_telemetry.addData("DriveSubsystem Data:", "");

        robot.m_telemetry.addData("FWD: ", -gamepad1.left_stick_y);
        robot.m_telemetry.addData("STR: ", gamepad1.left_stick_x);
        robot.m_telemetry.addData("RCW: ", gamepad1.right_stick_x);

        robot.m_telemetry.addData("DriveSubsystem end", "\n");

        if ( RobotHardware.DebugMode ) {
            DebugMode();
        }
    }

    private void DebugMode(){
        robot.m_telemetry.addData("DriveSubsystem Debug Data:", "");

        /*robot.m_telemetry.addData("wa1: ", wa1);
        robot.m_telemetry.addData("wa2: ", wa2);
        robot.m_telemetry.addData("wa3: ", wa3);
        robot.m_telemetry.addData("wa4: ", wa4);
        robot.m_telemetry.addData("ws1: ", ws1);
        robot.m_telemetry.addData("ws2: ", ws2);
        robot.m_telemetry.addData("ws3: ", ws3);
        robot.m_telemetry.addData("ws4: ", ws4);*/

        robot.m_telemetry.addData("miliamps front right: ", robot.motorFrontRight.getCurrent(CurrentUnit.MILLIAMPS));
        robot.m_telemetry.addData("miliamps front left: ", robot.motorFrontLeft.getCurrent(CurrentUnit.MILLIAMPS));
        robot.m_telemetry.addData("miliamps back right: ", robot.motorBackRight.getCurrent(CurrentUnit.MILLIAMPS));

        robot.m_telemetry.addData("miliamps back left: ", robot.motorBackLeft.getCurrent(CurrentUnit.MILLIAMPS));

        robot.m_telemetry.addData("Absolute analog encoder front right", robot.encoderFrontRight.getCurrentPosition());

        robot.m_telemetry.addData("DriveSubsystem Debug end", "\n");
    }

}

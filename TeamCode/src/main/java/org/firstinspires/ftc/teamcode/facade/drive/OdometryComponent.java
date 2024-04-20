package org.firstinspires.ftc.teamcode.facade.drive;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;

public class OdometryComponent {

    private boolean TelemeteryEnabled = true;
    public static double X = 0.0d, Y = 0.0d, theta = 0.0d, Theta = 0.0d, botHeading;
    private RobotHardware robot;

    private double L = 31.5d;//left to right
    private double B = 17d;//centre to front

    private double delta_x = 0.0d, delta_y = 0.0d, delta_theta = 0.0d;

    private final double TPR = 8192.0d;
    private final double R = 1.75;//cm

    private final double CM_perTick = 2 * Math.PI * R/TPR;

    private final double baba = 8.3;

    private double n_left = 0.0d, n_right = 0.0d, n_front = 0.0d;
    private double new_nLeft = 0.0d, new_nRight = 0.0d, new_nFront = 0.0d;
    private Motor EncoderLeft, EncoderRight, EncoderFront;
    IMU imu;

    public OdometryComponent(RobotHardware robot){
        this.robot = robot;

        imu = this.robot.m_imu;
        imu.resetYaw();
        imu.initialize(this.robot.imu_parameters);

        EncoderLeft = new Motor(robot.m_HardwareMap, "motorFrontLeft");
        EncoderRight = new Motor(robot.m_HardwareMap, "motorIntake");
        EncoderFront  = new Motor(robot.m_HardwareMap, "motorFrontRight");


        EncoderLeft.resetEncoder();
        EncoderRight.resetEncoder();
        EncoderFront.resetEncoder();
        X = 0.0d; Y = 0.0d; theta = 0.0d;
        Theta= 0.0d;
    }

    public void update(){

        n_left = new_nLeft;
        n_right = new_nRight;
        n_front = new_nFront;

        new_nLeft = -EncoderLeft.getCurrentPosition();
        new_nRight = -EncoderRight.getCurrentPosition();
        new_nFront = EncoderFront.getCurrentPosition();

        double dn1 = new_nLeft - n_left;
        double dn2 = new_nRight - n_right;
        double dn3 = new_nFront - n_front;

        delta_x = CM_perTick * (dn1 + dn2) / 2.0;
        delta_y = CM_perTick * (dn3 - ( dn2 - dn1 ) * B / L);
        delta_theta = CM_perTick * (dn2 - dn1) / L;

        X += delta_x * Math.cos(theta) - delta_y * Math.sin(theta);
        Y += delta_x * Math.sin(theta) + delta_y * Math.cos(theta);
        theta += delta_theta;
        if(theta<0) theta=theta+2*Math.PI;
        if(theta>2*Math.PI) theta=theta-2*Math.PI;

        botHeading =-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        if(0<=botHeading && botHeading<=Math.PI/2)
            botHeading=Math.PI/2-botHeading;
        else if(Math.PI/2<=botHeading && botHeading<=Math.PI)
            botHeading=2*Math.PI+Math.PI/2-botHeading;
        else
            botHeading=-botHeading+Math.PI/2;
        botHeading=botHeading-Math.PI/2;
        if(botHeading<0) botHeading=botHeading+2*Math.PI;

        theta=botHeading;
        Theta = Math.toDegrees(theta);
        AddTelemetry();
    }

    private void AddTelemetry() {
        if (!TelemeteryEnabled)
            return;

        robot.m_telemetry.addData("Odometry Data:", "");

        robot.m_telemetry.addData("x: ", X);
        robot.m_telemetry.addData("y: ", Y);
        robot.m_telemetry.addData("theta: ", theta);

        robot.m_telemetry.addData("left", new_nLeft);
        robot.m_telemetry.addData("right", new_nRight);

        robot.m_telemetry.addData("front", new_nFront);


        robot.m_telemetry.addData("Odometry end", "\n");

        if (RobotHardware.DebugMode) {
            //DebugMode();
        }

    }
    private void DebugMode(){
        robot.m_telemetry.addData("Odometry Debug Data:", "");


        robot.m_telemetry.addData("Odometry Debug end", "\n");
    }
}
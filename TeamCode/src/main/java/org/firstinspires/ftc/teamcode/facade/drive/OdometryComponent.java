package org.firstinspires.ftc.teamcode.facade.drive;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.hardware.motors.Motor;


import org.firstinspires.ftc.teamcode.facade.RobotHardware;

public class OdometryComponent {

    private boolean TelemeteryEnabled = true;
    public static double X = 0.0d, Y = 0.0d, ThetaTemp = 0.0d, Theta = 0.0d;
    private RobotHardware robot;

    private double L = 31.5d;//left to right
    private double B = 16.3d;//centre to front

    private double delta_x = 0.0d, delta_y = 0.0d, delta_theta = 0.0d;

    private final double TPR = 8192.0d;
    private final double R = 2.4;//cm

    private final double CM_perTick = 2 * PI * R/TPR;

    private final double baba = 8.55;

    private double n_left = 0.0d, n_right = 0.0d, n_front = 0.0d;
    private double new_nLeft = 0.0d, new_nRight = 0.0d, new_nFront = 0.0d;
    private Motor EncoderLeft, EncoderRight, EncoderFront;

    public OdometryComponent(RobotHardware robot){
        this.robot = robot;

        EncoderLeft = new Motor(robot.m_HardwareMap, "motorFrontLeft");
        EncoderRight = new Motor(robot.m_HardwareMap, "motorFrontRight");
        EncoderFront  = new Motor(robot.m_HardwareMap, "motorBackRight");


        EncoderLeft.resetEncoder();
        EncoderRight.resetEncoder();
        EncoderFront.resetEncoder();
        X = 0.0d; Y = 0.0d; ThetaTemp = 0.0d; Theta = 0.0d;
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

      double th = ThetaTemp + (delta_theta / 2.0d);

      X += delta_x * Math.cos(th) - delta_y * Math.sin(th);
      Y += delta_x * Math.sin(th) + delta_y * Math.cos(th);
      ThetaTemp += delta_theta;
      Theta = ((ThetaTemp * 360 /baba) + 360) % 360;
      Theta+=360;
      Theta%=360;
      AddTelemetry();
    }

    private void AddTelemetry() {
        if (!TelemeteryEnabled)
            return;

        robot.m_telemetry.addData("Odometry Data:", "");

        robot.m_telemetry.addData("x: ", X);
        robot.m_telemetry.addData("y: ", Y);
        robot.m_telemetry.addData("theta: ", ThetaTemp);
        robot.m_telemetry.addData("thetaBA: ", Theta);

        robot.m_telemetry.addData("left", new_nLeft);
        robot.m_telemetry.addData("right", new_nRight);

        robot.m_telemetry.addData("front", new_nFront);


        robot.m_telemetry.addData("Odometry end", "\n");

        if (RobotHardware.DebugMode) {
            DebugMode();
        }

    }
    private void DebugMode(){
        robot.m_telemetry.addData("Odometry Debug Data:", "");


        robot.m_telemetry.addData("Odometry Debug end", "\n");
    }
}

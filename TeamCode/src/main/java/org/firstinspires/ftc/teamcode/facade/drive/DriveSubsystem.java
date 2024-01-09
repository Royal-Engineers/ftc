package org.firstinspires.ftc.teamcode.facade.drive;

import static java.lang.Math.atan2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;

@Config

public class DriveSubsystem {

    public boolean TelemeteryEnabled = true;
    private RobotHardware robot;
    private double L = 1.6, W=1.2;
    private double R = Math.hypot(L/2, W/2);
    private swerveModule moduleFrontRight, moduleFrontLeft, moduleBackLeft, moduleBackRight;
    private Gamepad gamepad1;
    private double ws1, ws2, ws3, ws4, wa1, wa2, wa3, wa4;
    private double vx,vy,w,lastvx,lastvy,lastw,lastbotHeading,ax,ay,e,botHeading;
    private double[] vm=new double[5];
    private double[] am=new double[5];
    private double[] wm=new double[5];
    private double[] thetam=new double[5];
    private double[] rx=new double[5];
    private double[] ry=new double[5];
    private double[] alpha=new double[5];
    private double[] beta=new double[5];
    private static ElapsedTime dt=new ElapsedTime();
    IMU imu;

    public DriveSubsystem(RobotHardware robot){
        this.robot = robot;
        moduleBackRight = robot.moduleBackRight;
        moduleBackLeft = robot.moduleBackLeft;

        moduleFrontRight = robot.moduleFrontRight;
        moduleFrontLeft = robot.moduleFrontLeft;

        gamepad1 = robot.m_gamepad1;

        imu = this.robot.m_imu;
        imu.resetYaw();
        imu.initialize(this.robot.imu_parameters);

        rx[0]=W/2; ry[0]=L/2; beta[0]=atan2(ry[0],rx[0]); if(beta[0]<0) beta[0]=beta[0]+2*Math.PI;
        rx[1]=-W/2; ry[1]=L/2; beta[1]=atan2(ry[1],rx[1]); if(beta[1]<0) beta[1]=beta[1]+2*Math.PI;
        rx[2]=-W/2; ry[2]=-L/2; beta[2]=atan2(ry[2],rx[2]); if(beta[2]<0) beta[2]=beta[2]+2*Math.PI;
        rx[3]=W/2; ry[3]=-L/2; beta[3]=atan2(ry[3],rx[3]); if(beta[3]<0) beta[3]=beta[3]+2*Math.PI;
        lastvx=0; lastvy=0;
        lastw=0; lastbotHeading=0;
        dt.reset();
    }
    public void setKinematics(int cnt)
    {
        beta[cnt]=beta[cnt]+botHeading-lastbotHeading; if(beta[cnt]<0) beta[cnt]=beta[cnt]+2*Math.PI;
        rx[cnt]=Math.cos(beta[cnt]); ry[cnt]=Math.sin(beta[cnt]);

        double vmx=vx-ry[cnt]*w;
        double vmy=vy+rx[cnt]*w;

        vm[cnt]=Math.sqrt(vmx*vmx+vmy*vmy);
        thetam[cnt]=atan2(vmy,vmx);
        if(thetam[cnt]<0) thetam[cnt]=thetam[cnt]+2*Math.PI;

        double amx=ax-e*ry[cnt]-w*w*rx[cnt];
        double amy=ay+e*rx[cnt]-w*w*ry[cnt];

        am[cnt]=Math.cos(thetam[cnt])*amx+Math.sin(thetam[cnt])*amy;
        wm[cnt]=-Math.sin(thetam[cnt])*amx/vm[cnt]+Math.cos(thetam[cnt])*amy/vm[cnt];

        double wr=(botHeading-lastbotHeading)/dt.time();
        wm[cnt]=wm[cnt]-wr;
        thetam[cnt]=thetam[cnt]-botHeading;
        if(thetam[cnt]<0) thetam[cnt]=thetam[cnt]+2*Math.PI;

        vm[cnt]=vm[cnt]+am[cnt]*dt.time();
        thetam[cnt]=thetam[cnt]+wm[cnt]*dt.time();
        if(thetam[cnt]<0) thetam[cnt]+=2*Math.PI;

        thetam[cnt]=Math.toDegrees(thetam[cnt]);
        if(0<=thetam[cnt] && thetam[cnt]<=90)
        {
            thetam[cnt]=90-thetam[cnt];
        }
        else if(90<thetam[cnt] && thetam[cnt]<=270)
        {
            thetam[cnt]=-(thetam[cnt]-90);
        }
        else
        {
            thetam[cnt]=360-thetam[cnt]+90;
        }
    }
    public void UpdateGamepad(){

        vx=gamepad1.left_stick_x;
        vy=-gamepad1.left_stick_y;
        w=-gamepad1.right_stick_x;

        ax=(vx-lastvx)/dt.time();
        ay=(vy-lastvy)/dt.time();
        e=(w-lastw)/dt.time();

        botHeading =-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        if(0<=botHeading && botHeading<=Math.PI/2)
            botHeading=Math.PI/2-botHeading;
        else if(Math.PI/2<=botHeading && botHeading<=Math.PI)
            botHeading=2*Math.PI+Math.PI/2-botHeading;
        else
            botHeading=-botHeading+Math.PI/2;
        botHeading=botHeading-Math.PI/2;
        if(botHeading<0) botHeading=botHeading+2*Math.PI;

        setKinematics(0); setKinematics(1);
        setKinematics(2); setKinematics(3);

        double maxx=vm[0];
        if(vm[1]>maxx) maxx=vm[1]; if(vm[2]>maxx) maxx=vm[2]; if(vm[3]>maxx) maxx=vm[3];
        if(maxx>1)
        {
            vm[0]=vm[0]/maxx; vm[1]=vm[1]/maxx;
            vm[2]=vm[2]/maxx; vm[3]=vm[3]/maxx;
        }

        moduleFrontRight.drive(vm[0],thetam[0]);
        moduleFrontLeft.drive(vm[1],thetam[1]);
        moduleBackLeft.drive(vm[2],thetam[2]);
        moduleBackRight.drive(vm[3],thetam[3]);

        lastvx=vx; lastvy=vy; lastw=w;
        lastbotHeading=botHeading;
        dt.reset();

        AddTelemetry();
    }

    public void UpdateAuto(double fwd, double str, double rcw){

        vx=str;
        vy=fwd;
        w=-rcw;

        ax=(vx-lastvx)/dt.time();
        ay=(vy-lastvy)/dt.time();
        e=(w-lastw)/dt.time();

        botHeading =-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        if(0<=botHeading && botHeading<=Math.PI/2)
            botHeading=Math.PI/2-botHeading;
        else if(Math.PI/2<=botHeading && botHeading<=Math.PI)
            botHeading=2*Math.PI+Math.PI/2-botHeading;
        else
            botHeading=-botHeading+Math.PI/2;
        botHeading=botHeading-Math.PI/2;
        if(botHeading<0) botHeading=botHeading+2*Math.PI;

        setKinematics(0); setKinematics(1);
        setKinematics(2); setKinematics(3);

        double maxx=vm[0];
        if(vm[1]>maxx) maxx=vm[1]; if(vm[2]>maxx) maxx=vm[2]; if(vm[3]>maxx) maxx=vm[3];
        if(maxx>1)
        {
            vm[0]=vm[0]/maxx; vm[1]=vm[1]/maxx;
            vm[2]=vm[2]/maxx; vm[3]=vm[3]/maxx;
        }

        moduleFrontRight.drive(vm[0],thetam[0]);
        moduleFrontLeft.drive(vm[1],thetam[1]);
        moduleBackLeft.drive(vm[2],thetam[2]);
        moduleBackRight.drive(vm[3],thetam[3]);

        lastvx=vx; lastvy=vy; lastw=w;
        lastbotHeading=botHeading;
        dt.reset();

        AddTelemetry();


    }
    private void AddTelemetry(){
        if ( !TelemeteryEnabled )
            return;

        robot.m_telemetry.addData("DriveSubsystem Data:", "");

        /*robot.m_telemetry.addData("FWD: ", -gamepad1.left_stick_y);
        robot.m_telemetry.addData("STR: ", gamepad1.left_stick_x);
        robot.m_telemetry.addData("RCW: ", gamepad1.right_stick_x);

        robot.m_telemetry.addData("DriveSubsystem end", "\n");

        if ( RobotHardware.DebugMode ) {
            DebugMode();
        }*/

        robot.m_telemetry.addData("dt: ",dt.time());
        robot.m_telemetry.addData("vx: ",vx);
        robot.m_telemetry.addData("vy: ",vy);
        robot.m_telemetry.addData("w: ",w);

        robot.m_telemetry.addData("rx0: ",rx[0]);
        robot.m_telemetry.addData("ry0: ",ry[0]);
        robot.m_telemetry.addData("rx1: ",rx[1]);
        robot.m_telemetry.addData("ry1: ",ry[1]);
        robot.m_telemetry.addData("rx2: ",rx[2]);
        robot.m_telemetry.addData("ry2: ",ry[2]);
        robot.m_telemetry.addData("rx3: ",rx[3]);
        robot.m_telemetry.addData("ry3: ",ry[3]);

        robot.m_telemetry.addData("vm0: ",vm[0]);
        robot.m_telemetry.addData("vm1: ",vm[1]);
        robot.m_telemetry.addData("vm2: ",vm[2]);
        robot.m_telemetry.addData("vm3: ",vm[3]);

        robot.m_telemetry.addData("thetam0: ",thetam[0]);
        robot.m_telemetry.addData("thetam1: ",thetam[1]);
        robot.m_telemetry.addData("thetam2: ",thetam[2]);
        robot.m_telemetry.addData("thetam3: ",thetam[3]);
        robot.m_telemetry.addData("unghi ", botHeading);

        DebugMode();
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

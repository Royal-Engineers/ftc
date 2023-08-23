package org.firstinspires.ftc.teamcode.team20936.ftc;



import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.team20936.OpenCv.GetFrameImage;
import org.firstinspires.ftc.teamcode.team20936.Utils.globals;
import org.opencv.core.Mat;

import java.util.Calendar;

public class CameraUtil {

    private enum ConeClashStages
    {
        ResetStage,
        DetectCone,
        GotoCone,
        AlignToCone,
        CatchCone,

        DepositCone,
        neutral
    }

    private ConeClashStages camera_stage = ConeClashStages.DetectCone;

    private Robot m_robot;

    private GetFrameImage m_pipeline;

    private boolean HasCone;


    public CameraUtil() {
        m_robot = Robot.getInstance();
        m_pipeline = m_robot.m_InputPipeline;
        HasCone = m_pipeline.hasCon;
    }

    private void KeepDistanceToCone(double distmin, double distmax){
        if (   distantaCon > distmax )
            OutputFata = 0.05;
        else if ( distantaCon < distmin )
            OutputFata = -0.05;

        OutputRotatie = P * (error+42)* 0.7 + D * errorRate;

        if ( OutputRotatie > 0 )
            OutputRotatie = Math.min(1, OutputRotatie);
        else
            OutputRotatie = Math.max(-1, OutputRotatie);

        if ( Math.abs(OutputRotatie) < 0.1 )
            OutputRotatie = 0;


    }
    private void Reset(){
        caught = false;
        m_robot.m_ArmMotor.setTargetPosition(0);
        m_robot.m_ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_robot.m_ArmMotor.setPower(0.5);

        m_robot.m_RecoveryServo.setPosition(0.99);
        m_robot.m_WristServo.setPosition(0.37);
        m_robot. m_WristRotationServo.setPosition(0.4);
        m_robot.m_LatchServo.setPosition(0.1);
    }

    public void UpdateData(){
        LastCameraFrame = globals.CameraFrame;
        HasCone = m_pipeline.hasCon;
        distantaCon = m_pipeline.height - m_pipeline.posDown;
        time = Calendar.getInstance().getTimeInMillis();
        dt = time - lastTimestamp;
        error = m_pipeline.proc;
        if ( dt > 0.001)
            errorRate = (error - lastError)/dt;
        else
            errorRate = 0;

        m_robot.m_telemetry.addData("Stage:", camera_stage);
        m_robot.m_telemetry.addData("Distance: ", distantaCon);
        m_robot.m_telemetry.addData("HasCone: ", HasCone);
        m_robot.m_telemetry.addData("Error: ", error);
        m_robot.m_telemetry.update();

    }
    public static double P = 0.005, I = 0, D = 0.001;

    Mat LastCameraFrame = new Mat();

    double lastTimestamp = Calendar.getInstance().getTimeInMillis(), dt, time;
    double distantaCon;
    double error, errorRate, lastError = 0;

    double OutputRotatie = 0, OutputFata = 0;

    public void update(){
        UpdateData();

        if ( camera_stage == ConeClashStages.neutral)
            return;

        if ( HasCone == false && camera_stage != ConeClashStages.DepositCone){
           camera_stage = ConeClashStages.DetectCone;
           Reset();
        }
        OutputFata = 0;
        OutputRotatie = 0;

        switch (camera_stage){
            case DetectCone:
                UpdateStage1();
                break;
            case GotoCone:
                UpdateStage2();
                break;
            case AlignToCone:
                UpdateStage3();
                break;
            case CatchCone:
                UpdateStage4();
                break;
            case DepositCone:
                UpdateStage5();
                break;
            default:
                break;
        }
        lastTimestamp = time;
        lastError = error;
        m_robot.mecDrive.driveRobotCentric(0, OutputFata, -OutputRotatie);
    }




    private void UpdateStage1(){
        if ( HasCone )
            camera_stage = ConeClashStages.GotoCone;
        OutputRotatie = 0.3;
    }

    public static double DistantaConMinima = 130, DistantaConMaxima = 135, TolerantaDistanta = 5;

    private void UpdateStage2(){

        if (HasCone == false) {
            camera_stage = ConeClashStages.DetectCone;
        return;
        }

        if ( DistantaConMinima <= distantaCon && distantaCon <= DistantaConMaxima ) {
            camera_stage = ConeClashStages.AlignToCone;
            return;
        }


        if ( Math.abs(error) < 25  && distantaCon > DistantaConMaxima )
            OutputFata = 0.4;
        else if ( Math.abs(error) < 25 && distantaCon < DistantaConMinima )
            OutputFata = -0.4;

        OutputRotatie = P * error  + D * errorRate;

        if ( OutputRotatie > 0 )
            OutputRotatie = Math.min(1, OutputRotatie);
        else
            OutputRotatie = Math.max(-1, OutputRotatie);

        if ( Math.abs(OutputRotatie) < 0.1 )
            OutputRotatie = 0;

    }

    private void UpdateStage3(){
        KeepDistanceToCone(DistantaConMinima, DistantaConMaxima);
        double newError = error + 42;
        if (Math.abs(newError) > 10 || OutputFata > 0.01)
            return;

        if ( Math.abs(m_robot.m_ArmMotor.getCurrentPosition() - (m_robot.poz0-400)) > 10) {
            m_robot.m_ArmMotor.setTargetPosition(m_robot.poz0 - 400);
            m_robot.m_ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m_robot.m_ArmMotor.setPower(0.25);


            m_robot.m_WristRotationServo.setPosition(0.413);
            m_robot.m_WristServo.setPosition(0.435);
        }
        else
        {
            camera_stage = ConeClashStages.CatchCone;
        }

    }
    private boolean caught = false;
    private void UpdateStage4(){

        if(m_robot.m_ClawDistanceSensor.getDistance(DistanceUnit.MM) < 15) {

            m_robot.m_ClawServo.setPosition(0.0);
            camera_stage = ConeClashStages.DepositCone;
        }
        else
            OutputFata = 0.1;
    }

    private void UpdateStage5(){
        if ( m_robot.m_ClawServo.getPosition() < 0.1) {
            m_robot.m_ArmMotor.setTargetPosition(m_robot.poz0 - 80);
            m_robot.m_ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m_robot.m_ArmMotor.setPower(0.75);

            caught = true;
        }
        if ( caught &&Math.abs( m_robot.m_ArmMotor.getCurrentPosition() - (m_robot.poz0 - 80)) < 10)
        {
            camera_stage = ConeClashStages.neutral;
            m_robot.m_WristServo.setPosition(0.17);
            m_robot. m_WristRotationServo.setPosition(0.4);
        }
    }



}

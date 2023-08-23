package org.firstinspires.ftc.teamcode.team20936.ftc;


import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.team20936.OpenCv.GetFrameImage;
import org.firstinspires.ftc.teamcode.team20936.Utils.globals;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Robot {

    public GetFrameImage m_InputPipeline;
    private static volatile Robot m_instance = null;

    public Telemetry m_telemetry;

    private Motor fL, fR, bR, bL;

    public final MecanumDrive mecDrive;

    public final DistanceSensor m_ClawDistanceSensor;

    public final ServoEx m_WristServo;

    public final ServoEx m_WristRotationServo;

    public final ServoEx m_ClawServo;

    public final ServoEx m_LatchServo;

    public final ServoEx m_RecoveryServo;

    public final DcMotor m_ArmMotor, m_LeftMotor, m_RightMotor;

    public int poz0;
    public OpenCvCamera m_camera;
    public Robot(Telemetry telemetry, HardwareMap hardwareMap){
        m_telemetry = telemetry;
        m_WristServo = new SimpleServo( hardwareMap,"servo_brat2", globals.servo_min_position, globals.servo_max_position);
        m_WristRotationServo = new SimpleServo(hardwareMap, "servo_brat3", globals.servo_min_position, globals.servo_max_position);
        m_ClawServo= new SimpleServo(hardwareMap, "servo_gheara", globals.servo_min_position, globals.servo_max_position);
        m_LatchServo = new SimpleServo(hardwareMap,"servo_deposit", globals.servo_min_position, globals.servo_max_position);
        m_RecoveryServo = new SimpleServo(hardwareMap, "servo_4", globals.servo_min_position, globals.servo_max_position);

        m_ArmMotor = hardwareMap.get(DcMotorEx.class, "rev_hd_brat");
        poz0 = m_ArmMotor.getCurrentPosition();

        m_ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_ClawDistanceSensor = hardwareMap.get(DistanceSensor.class, "senzor_gheara");

        fR = new Motor(hardwareMap, "motorFrontRight", Motor.GoBILDA.RPM_312); fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fL = new Motor(hardwareMap, "motorFrontLeft", Motor.GoBILDA.RPM_312); fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR = new Motor(hardwareMap, "motorBackRight", Motor.GoBILDA.RPM_312); bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL = new Motor(hardwareMap, "motorBackLeft", Motor.GoBILDA.RPM_312); bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        mecDrive = new MecanumDrive(fL, fR, bL, bR);

        m_LeftMotor = hardwareMap.get(DcMotorEx.class, "liftStanga");
        m_LeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m_LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_RightMotor = hardwareMap.get(DcMotorEx.class, "liftDreapta");
        m_RightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m_RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
         m_InputPipeline = new GetFrameImage(telemetry);
        m_camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        m_camera.setPipeline(m_InputPipeline);

        m_camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                m_camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        telemetry.setMsTransmissionInterval(100);

        m_ArmMotor.setTargetPosition(poz0);
        m_ArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        m_ArmMotor.setPower(0.25);
        m_RecoveryServo.setPosition(0.99);
        m_WristServo.setPosition(0.37);
        m_WristRotationServo.setPosition(0.4);
        m_ClawServo.setPosition(0.3);
        m_LatchServo.setPosition(0.1);
        m_instance = this;
    }

    public static Robot getInstance() {
        assert (m_instance != null);
        return m_instance;
    }




}

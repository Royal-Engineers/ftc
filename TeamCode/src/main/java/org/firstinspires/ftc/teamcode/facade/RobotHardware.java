package org.firstinspires.ftc.teamcode.facade;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.facade.drive.absoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.facade.drive.swerveModule;
import org.firstinspires.ftc.teamcode.facade.intake.BombaSexy;
import org.firstinspires.ftc.teamcode.facade.intake.BombasticLift;
import org.openftc.easyopencv.OpenCvCamera;

public class RobotHardware {

    public final static boolean DebugMode = true;
    private static RobotHardware instance = null;

    public  OpenCvCamera camera;


    public IMU m_imu;

    public IMU.Parameters imu_parameters;
    //swerve
    public DcMotorEx motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft;
    public DcMotorEx motorIntake;

    public DcMotorEx m_LiftMainMotor, m_LiftInvMotor;
    private CRServo servoFrontRight, servoFrontLeft, servoBackRight, servoBackLeft;
    public absoluteAnalogEncoder encoderFrontRight, encoderFrontLeft, encoderBackLeft, encoderBackRight;
    public AnalogInput aencoderFrontRight, aencoderFrontLeft, aencoderBackRight, aencoderBackLeft;
    public swerveModule moduleFrontRight, moduleFrontLeft, moduleBackLeft, moduleBackRight;


    //utils
    public Telemetry m_telemetry;
    public HardwareMap m_HardwareMap;

    //control teleop
    public GamepadEx controller1, controller2;
    public Gamepad m_gamepad1, m_gamepad2;

    //odometers
    public DcMotor EncoderLeft, EncoderRight, EncoderFront;

    public BombasticLift m_Lift;

    public BombaSexy m_BombaSexy;

    private ServoEx m_BombasticServo1, m_SexyServo2;
    private RobotHardware(){
    }

    public void init(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwaremap){
        this.m_HardwareMap = hardwaremap;
        this.m_telemetry = telemetry;

        motorFrontRight = m_HardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorFrontLeft = m_HardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackRight = m_HardwareMap.get(DcMotorEx.class, "motorBackRight");
        motorBackLeft = m_HardwareMap.get(DcMotorEx.class, "motorBackLeft");

        motorIntake = m_HardwareMap.get(DcMotorEx.class, "motorIntake");

        m_LiftMainMotor = m_HardwareMap.get(DcMotorEx.class, "LiftMainMotor");
        m_LiftInvMotor = m_HardwareMap.get(DcMotorEx.class, "LiftInvertedMotor");

        InitializeMotors();

        m_Lift = new BombasticLift(this, m_telemetry, m_LiftMainMotor, m_LiftInvMotor, 0, 840);

        servoFrontRight = m_HardwareMap.get(CRServo.class, "servoFrontRight");
        servoFrontLeft = m_HardwareMap.get(CRServo.class, "servoFrontLeft");
        servoBackRight = m_HardwareMap.get(CRServo.class, "servoBackRight");
        servoBackLeft = m_HardwareMap.get(CRServo.class, "servoBackLeft");

        aencoderFrontRight = m_HardwareMap.get(AnalogInput.class, "encoderFrontRight");
        aencoderFrontLeft = m_HardwareMap.get(AnalogInput.class, "encoderFrontLeft");
        aencoderBackRight = m_HardwareMap.get(AnalogInput.class, "encoderBackRight");
        aencoderBackLeft = m_HardwareMap.get(AnalogInput.class, "encoderBackLeft");
        encoderFrontRight = new absoluteAnalogEncoder(aencoderFrontRight, 53, true);
        encoderFrontLeft = new absoluteAnalogEncoder(aencoderFrontLeft, 205, true);
        encoderBackLeft = new absoluteAnalogEncoder(aencoderBackLeft, 207, true);
        encoderBackRight = new absoluteAnalogEncoder(aencoderBackRight, 225, true);

        moduleFrontRight = new swerveModule(motorFrontRight, servoFrontRight, encoderFrontRight, m_telemetry);
        moduleFrontLeft = new swerveModule(motorFrontLeft, servoFrontLeft, encoderFrontLeft, m_telemetry);
        moduleBackLeft = new swerveModule(motorBackLeft, servoBackLeft, encoderBackLeft, m_telemetry);
        moduleBackRight = new swerveModule(motorBackRight, servoBackRight, encoderBackRight, m_telemetry);
        m_gamepad1 = gamepad1;
        m_gamepad2 = gamepad2;

        controller1 = new GamepadEx(m_gamepad1);
        controller2 = new GamepadEx(m_gamepad2);

        EncoderLeft =  motorFrontLeft;
        EncoderRight = motorFrontRight;
        EncoderFront = motorBackRight;

        m_BombasticServo1 = new SimpleServo(m_HardwareMap, "BombasticServo1", 0, 1);
        m_SexyServo2 = new SimpleServo(m_HardwareMap, "BombasticServo2", 0, 1);

        m_BombasticServo1.setPosition(0.2);
        m_SexyServo2.setPosition(0.13);

        m_BombaSexy = new BombaSexy(this, m_BombasticServo1, m_SexyServo2, 0.2, 0.13);

        m_imu = m_HardwareMap.get(IMU.class, "imu");
        imu_parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // int cameraMonitorViewId = m_HardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", m_HardwareMap.appContext.getPackageName());
       // camera = OpenCvCameraFactory.getInstance().createWebcam(m_HardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

/*        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });*/
    }

    public static RobotHardware getInstance(){
        RobotHardware result = instance;
        if ( result != null )
            return result;
        synchronized (RobotHardware.class) {
            if (instance == null)
                instance = new RobotHardware();
        }
        return instance;
    }

    private void InitializeMotors(){

        m_LiftInvMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m_LiftInvMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_LiftInvMotor.setTargetPositionTolerance(10);

        m_LiftMainMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m_LiftMainMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_LiftMainMotor.setTargetPositionTolerance(10);

        motorFrontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    public void Update()
    {
        m_Lift.UpdateTelemetry();
    }
}

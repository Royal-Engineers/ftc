package org.firstinspires.ftc.teamcode.team20936.auto;

import org.firstinspires.ftc.teamcode.team20936.auto.subsystems.CommandGroups;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.team20936.auto.autoCommands.BasicCommands.SorinMakesBetterWaitCommands;
import org.firstinspires.ftc.teamcode.team20936.auto.autoCommands.BasicCommands.GotoConstantHeading;
import org.firstinspires.ftc.teamcode.team20936.auto.autoCommands.BasicCommands.GotoLinearHeading;
import org.firstinspires.ftc.teamcode.team20936.auto.subsystems.Lift;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.team20936.auto.subsystems.Singleton_AutoSubsystem;


import java.util.ArrayList;

@Autonomous
public class autostanga extends CommandOpMode
{

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    public static boolean isFinisheed = false;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    double MIN_POSITION = 0;
    double MAX_POSITION = 1;
    AprilTagDetection tagOfInterest = null;
    private ElapsedTime timer;
    private Pose2d startPose;
    private DistanceSensor sensorDistanta_intake;
    private  ServoEx m_wrist;
    private  ServoEx m_wristRev;
    private  ServoEx m_claw;
    private  ServoEx m_latch;
    public  SampleMecanumDrive m_drive;
    private  DcMotor m_armRev;
    private  Lift m_lift;
    Singleton_AutoSubsystem m_subsystem;

    @Override
    public void initialize()
    {

        CommandScheduler.getInstance().reset();
        m_drive = new SampleMecanumDrive(hardwareMap);
        Lift lift = new Lift();
        lift.init(hardwareMap);

        m_wrist = new SimpleServo( hardwareMap,"servo_brat2", MIN_POSITION, MAX_POSITION);
        m_wristRev = new SimpleServo(hardwareMap, "servo_brat3", MIN_POSITION, MAX_POSITION);
        m_claw= new SimpleServo(hardwareMap, "servo_gheara", MIN_POSITION, MAX_POSITION);
        m_latch = new SimpleServo(hardwareMap,"servo_deposit", MIN_POSITION, MAX_POSITION);




        m_armRev = hardwareMap.dcMotor.get("rev_hd_brat");
        m_armRev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_subsystem = Singleton_AutoSubsystem.createInstance(m_wrist, m_wristRev, m_claw,
                m_latch, m_drive, m_armRev, lift);
        timer = new ElapsedTime();
        startPose = new Pose2d(0, 0, 0);
        sensorDistanta_intake = hardwareMap.get(DistanceSensor.class, "senzor_gheara");



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
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
        });

        telemetry.setMsTransmissionInterval(100);
        int cnt = 0 ;
        m_subsystem.setWristPos(Range.clip(0.133, 0, 1));
        m_subsystem.setWristRevPos(Range.clip(0.41, 0, 1));
        m_subsystem.setClawPos(Range.clip(0.22, 0, 1));

        m_subsystem.setLatchPos(Range.clip(0.23, 0, 1));

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new GotoConstantHeading(2, 0)));
        CommandScheduler.getInstance().run();
        while (!isStarted() && !isStopRequested() )
        {


            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            cnt++;
            sleep(20);
        }


    }


    @Override
    public void run() {

        if (!isFinisheed) {

            double ParcareX, ParcareY;
            if(tagOfInterest != null)
            {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            }
            else
            {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

            if(tagOfInterest == null || tagOfInterest.id == MIDDLE){
                ParcareX = 4;
                ParcareY = 55;
            }else if(tagOfInterest.id == LEFT){
                ParcareX = -20;
                ParcareY = 55;
            }else{
                ParcareX = 33;
                ParcareY = 55;
            }
            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                    new GotoLinearHeading(2, 51, -45),
                   CommandGroups.TransferConeGroup(8.5, 48.5, -45, 1010),

                    new GotoLinearHeading(-6.2, 52.9, 0),
                    CommandGroups.CatchConeGroup(-354),
                    new SorinMakesBetterWaitCommands(400),
                    CommandGroups.DepositGroup(),
                    new GotoLinearHeading(8, 51, -45),
                    CommandGroups.TransferConeGroup(9.5, 49),

                    new GotoLinearHeading(-5.1, 52.9, 0),
                    CommandGroups.CatchConeGroup(-385),
                    new SorinMakesBetterWaitCommands(400),
                    CommandGroups.DepositGroup(),
                    new GotoLinearHeading(8, 51, -45),
                    CommandGroups.TransferConeGroup(9.5, 49),

                    new GotoLinearHeading(-3.7, 53.1,0),
                    CommandGroups.CatchConeGroup(-400),
                    new SorinMakesBetterWaitCommands(400),
                    CommandGroups.DepositGroup(),
                    new SorinMakesBetterWaitCommands(200),
                    new GotoLinearHeading(8, 50.5, -45),

                    CommandGroups.TransferConeGroup(9.5, 49),

                    new GotoLinearHeading(9.5, 55, 90),
                    new GotoConstantHeading(ParcareX, ParcareY)
                    ));
        isFinisheed = true;




        }
        CommandScheduler.getInstance().run();




    }



    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}

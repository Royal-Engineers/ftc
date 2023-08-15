package org.firstinspires.ftc.teamcode.team20936.auto;

        import org.firstinspires.ftc.teamcode.team20936.auto.autoCommands.BasicCommands.SetArmRevPos;
        import org.firstinspires.ftc.teamcode.team20936.auto.autoCommands.BasicCommands.SetWristPos;
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
public class autodreapta extends CommandOpMode
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

    private ServoEx m_chestie;
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
        m_chestie = new SimpleServo(hardwareMap, "servo_4", MIN_POSITION, MAX_POSITION);
        m_chestie.setPosition(0.99);


        m_armRev = hardwareMap.dcMotor.get("rev_hd_brat");
        m_armRev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_subsystem = Singleton_AutoSubsystem.createInstance(m_wrist, m_wristRev, m_claw,
                m_latch, m_drive, m_armRev, lift, m_chestie);
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

            if(tagOfInterest == null || tagOfInterest.id == MIDDLE){ //  merge
                ParcareX = 7;
                ParcareY = 27;
            }else if(tagOfInterest.id == LEFT){
                ParcareX = -15;
                ParcareY = 27;
            }else{
                ParcareX = 28.5;
                ParcareY = 27;
            }//Doamne ajuta


            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                    new GotoConstantHeading(3.3, 0),
                    new GotoConstantHeading(3.3, 30),
                    new GotoLinearHeading(3.3, 28, 135),
                    new SorinMakesBetterWaitCommands(200),
                    CommandGroups.TransferConeGroupRight(-1, 31, 135, 1010),
                    new SorinMakesBetterWaitCommands(600),

                    new SetArmRevPos(5),
                    new SetWristPos(0.45),
                    new SorinMakesBetterWaitCommands(100),
                    new GotoLinearHeading(-0.5, 31, 90),
                    new SorinMakesBetterWaitCommands(400),
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

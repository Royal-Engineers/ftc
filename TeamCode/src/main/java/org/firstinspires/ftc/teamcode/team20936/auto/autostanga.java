package org.firstinspires.ftc.teamcode.team20936.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.team20936.auto.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.team20936.auto.subsystems.Lift;


import java.util.ArrayList;

@Autonomous
public class autostanga extends LinearOpMode
{

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

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

    AprilTagDetection tagOfInterest = null;



    @Override
    public void runOpMode() throws InterruptedException
    {
        ElapsedTime timer = new ElapsedTime();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);

        Lift lift = new Lift(); lift.init(hardwareMap);

        Servo wrist = hardwareMap.servo.get("servo_brat2");
        Servo wristRev = hardwareMap.servo.get("servo_brat3");
        Servo claw= hardwareMap.servo.get("servo_gheara");

        Servo latch = hardwareMap.servo.get("servo_deposit");

        DcMotor armRev = hardwareMap.dcMotor.get("rev_hd_brat"); armRev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DistanceSensor sensorDistanta_intake = hardwareMap.get(DistanceSensor.class, "senzor_gheara");


        double MIN_POSITION = 0;
        double MAX_POSITION = 1;

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

        final int poz0_rev = armRev.getCurrentPosition();


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {

            wrist.setPosition(Range.clip(0.133, 0, 1));
            wristRev.setPosition(Range.clip(0.41, 0, 1));
            claw.setPosition(Range.clip(0.22, 0, 1));

            latch.setPosition(Range.clip(0.23, 0, 1));




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
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */



        TrajectorySequence preload = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(2,0))
                .lineToLinearHeading(new Pose2d(2,52, Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    lift.setLiftPosition(1010);

                    armRev.setTargetPosition(poz0_rev-80);
                    armRev.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    armRev.setPower(0.45);
                })
                .lineToConstantHeading(new Vector2d(7 , 48)) // poz mediu
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> {
                    latch.setPosition(0.1);
                    lift.setLiftPosition(0);
                })
                .waitSeconds(0.6)
                .lineToLinearHeading(new Pose2d(-6, 53, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    armRev.setTargetPosition(poz0_rev-354);
                    armRev.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    armRev.setPower(0.25);
                    wrist.setPosition(0.32);
                    wristRev.setPosition(0.41);
                })
                .waitSeconds(0.5 )
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    claw.setPosition(0);
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->
                {
                    armRev.setTargetPosition(-45);
                    armRev.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    armRev.setPower(1);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->
                {
                    wristRev.setPosition(0.958);

                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->
                {
                    wrist.setPosition(0.617);
                })
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->
                {
                    claw.setPosition(0.14);
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    latch.setPosition(0.23);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->
                {

                    armRev.setTargetPosition(poz0_rev-80);
                    armRev.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    armRev.setPower(0.45);
                })
                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->
                {
                    wristRev.setPosition(0.4105);
                    wrist.setPosition(0.3);
                })


                .build();

        drive.followTrajectorySequence(preload);




        /* Update the telemetry */
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

        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == MIDDLE){

        }else if(tagOfInterest.id == LEFT){

        }else{

        }




        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
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

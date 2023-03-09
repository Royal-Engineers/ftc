package org.firstinspires.ftc.teamcode.team20936.auto;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);

        Lift lift = new Lift();
        Servo servo_gheara = hardwareMap.servo.get("servo_gheara");
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

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {

            servo_gheara.setPosition(Range.clip(0.06, MIN_POSITION, MAX_POSITION));

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
        lift.init(hardwareMap);

//        lift.moveLift(4050);
        // sab: mai trebuie lucrat
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
//        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .lineToConstantHeading(new Vector2d(0,2))
                .lineToLinearHeading(new Pose2d(6, 15, Math.toRadians(45)))
                .addDisplacementMarker(40, () -> {
                    ;
                                    lift.moveLift(4030);
                })
                .lineToConstantHeading(new Vector2d(6, 55))
                .lineToConstantHeading(new Vector2d(12, 57))
                .lineToConstantHeading(new Vector2d(14, 59.3))
                .waitSeconds(4)
                .addSpatialMarker(new Vector2d(14, 59.3), () -> {
                    ;
                                    servo_gheara.setPosition(0.13);
                })
                .waitSeconds(2)
                .lineToConstantHeading(new Vector2d(12.1, 57.8))
                .waitSeconds(2)
                .addDisplacementMarker(() -> {
                    ;
                                    lift.moveLift(500);
                })
                .waitSeconds(4)
//                                .lineToConstantHeading(new Vector2d(6, 56))
//                                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-10, 58, Math.toRadians(180)))
                .waitSeconds(3)
                .build();
        drive.followTrajectorySequence(traj1);
//        drive.setPoseEstimate(new Pose2d(6, 55, Math.toRadians(45)));
//        startPose = new Pose2d(6, 55, Math.toRadians(45));
//
//        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(startPose)
//                .lineToConstantHeading(new Vector2d(16, 62))
//                .build();
//        drive.followTrajectorySequence(traj2);

        sleep(2000);
        servo_gheara.setPosition(0.13);

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
//        if(tagOfInterest == null){
//            //default trajectory here if preferred
//        }else if(tagOfInterest.id == LEFT){
//            //left trajectory
//        }else if(tagOfInterest.id == MIDDLE){
//            //middle trajectory
//        }else{
//            //right trajectory
//        }

        //ajunge in patratul din fata high-ului

        /* sab
        if(tagOfInterest == null){
            Trajectory rotatie = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(-90)))
                    .addTemporalMarker(4, () ->{
                        //lift.moveLift(0);
                    })
                    .build();
            drive.followTrajectory(rotatie);

        }else{
            switch(tagOfInterest.id){
                case 1:
                {
                    Trajectory stanga = drive.trajectoryBuilder(startPose)
                            .lineToLinearHeading(new Pose2d(-30, 0, Math.toRadians(-90)))
                            .addTemporalMarker(4, () ->{
                                //lift.moveLift(0);
                            })
                            .build();
                    drive.followTrajectory(stanga);
                }
                    break;
                case 2:
                {
                    Trajectory rotatie = drive.trajectoryBuilder(startPose)
                            .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(-90)))
                            .addTemporalMarker(4, () ->{
                                //lift.moveLift(0);
                            })
                            .build();
                    drive.followTrajectory(rotatie);
                }
                    break;
                case 3:
                {

                    Trajectory dreapta = drive.trajectoryBuilder(startPose)
                            .lineToLinearHeading(new Pose2d(30, 0, Math.toRadians(-90)))
                            .addTemporalMarker(4, () ->{
                                //lift.moveLift(0);
                            })
                            .build();
                    drive.followTrajectory(dreapta);
                }
                    break;
            }
        }
        */ // sab end
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
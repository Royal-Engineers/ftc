package org.firstinspires.ftc.teamcode.team20936.auto;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.team20936.auto.subsystems.Lift;


import java.util.ArrayList;

@Autonomous
public class autodreapta extends LinearOpMode
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

        Servo servo_brat2 = hardwareMap.servo.get("servo_brat2");
        Servo servo_brat3 = hardwareMap.servo.get("servo_brat3");
        Servo servo_gheara = hardwareMap.servo.get("servo_gheara");

        Servo servo_deposit = hardwareMap.servo.get("servo_deposit");

        DcMotor rev_hd_brat = hardwareMap.dcMotor.get("rev_hd_brat"); rev_hd_brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        final int poz0_rev = rev_hd_brat.getCurrentPosition();


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {

            servo_brat2.setPosition(Range.clip(0.133, 0, 1));
            servo_brat3.setPosition(Range.clip(0.41, 0, 1));
            servo_gheara.setPosition(Range.clip(0.22, 0, 1));

            servo_deposit.setPosition(Range.clip(0.23, 0, 1));


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

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)

                .lineToConstantHeading(new Vector2d(5,0))
                .lineToConstantHeading(new Vector2d(5,47))
                .lineToLinearHeading(new Pose2d(3,50,100))

                // poz medium
                .addDisplacementMarker( () -> {
                    lift.moveLift(1710);

                    rev_hd_brat.setTargetPosition(poz0_rev-80);
                    rev_hd_brat.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rev_hd_brat.setPower(0.45);
                })

                .lineToLinearHeading(new Pose2d(3.1,50,-90))
                .waitSeconds(0.7)
                .lineToLinearHeading(new Pose2d(3,50,-90))
//                .addDisplacementMarker(() -> {
//                    servo_deposit.setPosition(0.1);
//                    lift.moveLift(0);
//                })
//                .lineToLinearHeading(new Pose2d(17,52,0))
//                .waitSeconds(0.1)
//                .lineToLinearHeading(new Pose2d(17.1,52,0))

                .build();
        drive.followTrajectorySequence(traj1);


        TrajectorySequence traj_med = drive.trajectorySequenceBuilder(new Pose2d(-8,50,0))

                .lineToLinearHeading(new Pose2d(-7.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_brat2.setPosition(0.35222);
                    rev_hd_brat.setTargetPosition(poz0_rev-350);
                    rev_hd_brat.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rev_hd_brat.setPower(0.45);
                })
                .lineToLinearHeading(new Pose2d(-7,52,0))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-7.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_gheara.setPosition(0);
                })
                .lineToLinearHeading(new Pose2d(-7,52,0))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-7.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_brat2.setPosition(0.4);

                    rev_hd_brat.setTargetPosition(poz0_rev-49);
                    rev_hd_brat.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rev_hd_brat.setPower(0.45);
                })

                .lineToLinearHeading(new Pose2d(-7,52,0))
                .waitSeconds(0.420)
                .lineToLinearHeading(new Pose2d(-7.1,52,0))

                .addDisplacementMarker(() -> {
                    servo_brat3.setPosition(0.99);
                })
                .lineToLinearHeading(new Pose2d(-7,52,0))
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(-7.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_brat2.setPosition(0.624444);
                })
                .lineToLinearHeading(new Pose2d(-7,52,0))
                .waitSeconds(0.450)
                .lineToLinearHeading(new Pose2d(-7.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_gheara.setPosition(Range.clip(0.15, 0, 1));
                    servo_deposit.setPosition(Range.clip(0.23, 0, 1));
                })
                .lineToLinearHeading(new Pose2d(-7,52,0))
                .waitSeconds(0.300)
                .lineToLinearHeading(new Pose2d(-7.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_brat2.setPosition(Range.clip(0.37, 0, 1));

                    rev_hd_brat.setTargetPosition(poz0_rev - 25);
                    rev_hd_brat.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rev_hd_brat.setPower(0.45);
                })
                .lineToLinearHeading(new Pose2d(-7,52,0))
                .waitSeconds(0.150)
                .lineToLinearHeading(new Pose2d(-7.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_gheara.setPosition(Range.clip(0.22,0,1));
                    rev_hd_brat.setTargetPosition(poz0_rev-25);
                    rev_hd_brat.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rev_hd_brat.setPower(0.45);
                    servo_brat2.setPosition(0.133);
                    servo_brat3.setPosition(0.41);
                })
                .lineToLinearHeading(new Pose2d(9,47,-45))
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(9.1,47,-45))

                // inapoi mid
                .addDisplacementMarker( () -> {
                    lift.moveLift(1010);
                })
                .lineToLinearHeading(new Pose2d(9,47,-45))
                .waitSeconds(0.7)
                .lineToLinearHeading(new Pose2d(9.1,47,-45))
                .addDisplacementMarker(() -> {
                    servo_deposit.setPosition(0.1);
                    lift.moveLift(0);
                })
                .lineToLinearHeading(new Pose2d(-8,52,0))
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(-8.1,52,0))

                .build();
       // drive.followTrajectorySequence(traj_med);


        TrajectorySequence traj_med1 = drive.trajectorySequenceBuilder(new Pose2d(-8,50,0))

                .lineToLinearHeading(new Pose2d(-7.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_brat2.setPosition(0.305555);
                    rev_hd_brat.setTargetPosition(poz0_rev-345);
                    rev_hd_brat.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rev_hd_brat.setPower(0.45);
                })
                .lineToLinearHeading(new Pose2d(-7,52,0))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-7.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_gheara.setPosition(0);
                })
                .lineToLinearHeading(new Pose2d(-7,52,0))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-7.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_brat2.setPosition(0.4);

                    rev_hd_brat.setTargetPosition(poz0_rev-49);
                    rev_hd_brat.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rev_hd_brat.setPower(0.45);
                })

                .lineToLinearHeading(new Pose2d(-7,52,0))
                .waitSeconds(0.420)
                .lineToLinearHeading(new Pose2d(-7.1,52,0))

                .addDisplacementMarker(() -> {
                    servo_brat3.setPosition(0.99);
                })
                .lineToLinearHeading(new Pose2d(-8,52,0))
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(-8.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_brat2.setPosition(0.624444);
                })
                .lineToLinearHeading(new Pose2d(-8,52,0))
                .waitSeconds(0.450)
                .lineToLinearHeading(new Pose2d(-8.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_gheara.setPosition(Range.clip(0.15, 0, 1));
                    servo_deposit.setPosition(Range.clip(0.23, 0, 1));
                })
                .lineToLinearHeading(new Pose2d(-8,52,0))
                .waitSeconds(0.300)
                .lineToLinearHeading(new Pose2d(-8.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_brat2.setPosition(Range.clip(0.37, 0, 1));

                    rev_hd_brat.setTargetPosition(poz0_rev - 25);
                    rev_hd_brat.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rev_hd_brat.setPower(0.45);
                })
                .lineToLinearHeading(new Pose2d(-8,52,0))
                .waitSeconds(0.150)
                .lineToLinearHeading(new Pose2d(-8.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_gheara.setPosition(Range.clip(0.22,0,1));
                    rev_hd_brat.setTargetPosition(poz0_rev-25);
                    rev_hd_brat.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rev_hd_brat.setPower(0.45);
                    servo_brat2.setPosition(0.133);
                    servo_brat3.setPosition(0.41);
                })
                .lineToLinearHeading(new Pose2d(9,47,-45))
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(9.1,47,-45))

                // inapoi mid
                .addDisplacementMarker( () -> {
                    lift.moveLift(1010);
                })
                .lineToLinearHeading(new Pose2d(9,47,-45))
                .waitSeconds(0.7)
                .lineToLinearHeading(new Pose2d(9.1,47,-45))
                .addDisplacementMarker(() -> {
                    servo_deposit.setPosition(0.1);
                    lift.moveLift(0);
                })
                .lineToLinearHeading(new Pose2d(-8,52,0))
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(-8.1,52,0))

                .build();
    //    drive.followTrajectorySequence(traj_med1);


        TrajectorySequence traj_med2 = drive.trajectorySequenceBuilder(new Pose2d(-8,52,0))

                .lineToLinearHeading(new Pose2d(-8.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_brat2.setPosition(0.31666666);
                    rev_hd_brat.setTargetPosition(poz0_rev-374);
                    rev_hd_brat.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rev_hd_brat.setPower(0.45);
                })
                .lineToLinearHeading(new Pose2d(-8,52,0))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-8.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_gheara.setPosition(0);
                })
                .lineToLinearHeading(new Pose2d(-8,52,0))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-8.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_brat2.setPosition(0.4);

                    rev_hd_brat.setTargetPosition(poz0_rev-49);
                    rev_hd_brat.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rev_hd_brat.setPower(0.45);
                })

                .lineToLinearHeading(new Pose2d(-8,52,0))
                .waitSeconds(0.420)
                .lineToLinearHeading(new Pose2d(-8.1,52,0))

                .addDisplacementMarker(() -> {
                    servo_brat3.setPosition(0.99);
                })
                .lineToLinearHeading(new Pose2d(-8,52,0))
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(-8.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_brat2.setPosition(0.624444);
                })
                .lineToLinearHeading(new Pose2d(-8,52,0))
                .waitSeconds(0.450)
                .lineToLinearHeading(new Pose2d(-8.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_gheara.setPosition(Range.clip(0.15, 0, 1));
                    servo_deposit.setPosition(Range.clip(0.23, 0, 1));
                })
                .lineToLinearHeading(new Pose2d(-8,52,0))
                .waitSeconds(0.300)
                .lineToLinearHeading(new Pose2d(-8.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_brat2.setPosition(Range.clip(0.37, 0, 1));

                    rev_hd_brat.setTargetPosition(poz0_rev - 25);
                    rev_hd_brat.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rev_hd_brat.setPower(0.45);
                })
                .lineToLinearHeading(new Pose2d(-8,52,0))
                .waitSeconds(0.150)
                .lineToLinearHeading(new Pose2d(-8.1,52,0))
                .addDisplacementMarker(() -> {
                    servo_gheara.setPosition(Range.clip(0.22,0,1));
                    rev_hd_brat.setTargetPosition(poz0_rev-25);
                    rev_hd_brat.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rev_hd_brat.setPower(0.45);
                    servo_brat2.setPosition(0.133);
                    servo_brat3.setPosition(0.41);
                })
                .lineToLinearHeading(new Pose2d(9,47,-45))
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(9.1,47,-45))

                // inapoi mid
                .addDisplacementMarker( () -> {
                    lift.moveLift(1010);
                })
                .lineToLinearHeading(new Pose2d(9,47,-45))
                .waitSeconds(0.7)
                .lineToLinearHeading(new Pose2d(9.1,47,-45))
                .addDisplacementMarker(() -> {
                    servo_deposit.setPosition(0.1);
                    lift.moveLift(0);
                })
                .lineToLinearHeading(new Pose2d(-8,52,0))
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(-8.1,52,0))

                .build();
        //drive.followTrajectorySequence(traj_med2);


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

        Pose2d startParc = new Pose2d(-8.1,52,0);
        TrajectorySequence parcare;
        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == MIDDLE){
            parcare = drive.trajectorySequenceBuilder(startParc)
                    // lucky guess poz 2
                    .lineToLinearHeading(new Pose2d(2,50,0))

                    .addDisplacementMarker(() -> {
                        servo_brat2.setPosition(Range.clip(0.37, 0, 1));
                        servo_brat3.setPosition(Range.clip(0.4, 0, 1));
                        servo_gheara.setPosition(Range.clip(0.22, 0, 1));
                        servo_deposit.setPosition(Range.clip(0.1, 0, 1));
                    })
                    .build();
        }else if(tagOfInterest.id == LEFT){
            parcare = drive.trajectorySequenceBuilder(startParc)

                    .lineToLinearHeading(new Pose2d(-15,50,0))

                    .addDisplacementMarker(() -> {
                        servo_brat2.setPosition(Range.clip(0.37, 0, 1));
                        servo_brat3.setPosition(Range.clip(0.4, 0, 1));
                        servo_gheara.setPosition(Range.clip(0.22, 0, 1));
                        servo_deposit.setPosition(Range.clip(0.1, 0, 1));
                    })
                    .build();
        }else{
            parcare = drive.trajectorySequenceBuilder(startParc)

                    .lineToLinearHeading(new Pose2d(25,50,0))

                    .addDisplacementMarker(() -> {
                        servo_brat2.setPosition(Range.clip(0.37, 0, 1));
                        servo_brat3.setPosition(Range.clip(0.4, 0, 1));
                        servo_gheara.setPosition(Range.clip(0.22, 0, 1));
                        servo_deposit.setPosition(Range.clip(0.1, 0, 1));
                    })
                    .build();
        }

        drive.followTrajectorySequence(parcare);


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

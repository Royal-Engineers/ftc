package org.firstinspires.ftc.teamcode.team20936.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.team20936.auto.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.team20936.auto.autoCommands.preloadTrajectoryLeft;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.team20936.auto.subsystems.Lift;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.depositSubsystem;
import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;

import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.clawGrab;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.clawRelease;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.latchClose;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.lowPos;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.latchOpen;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.manualControl;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.pickUp;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.raiseHigh;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.raiseMiddle;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.retract;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.setArmRevPos;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.setClawPos;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.setManualWristPos;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.setManualWristRevPos;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.setWristPos;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.setWristRevPos;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.transfer;


import java.util.ArrayList;

@Disabled
@Autonomous
public class commandBaseAutoLeft extends CommandOpMode {

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

    intakeSubsystem m_intakeSubsystem;
    depositSubsystem m_depositSubsystem;
    private DcMotorEx leftMotor, rightMotor, armRev;
    private ServoEx claw, wristRev, wrist, latch;
    private DistanceSensor sensorDistanta_intake;
    private SampleMecanumDrive drive;


    @Override
    public void initialize() {
        ElapsedTime timer = new ElapsedTime();

        CommandScheduler.getInstance().reset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        claw = new SimpleServo(hardwareMap, "servo_gheara", 0, 1);
        claw.setPosition(Range.clip(0.22, 0, 1));
        wristRev = new SimpleServo(hardwareMap, "servo_brat3", 0, 1);
        wristRev.setPosition(Range.clip(0.4, 0, 1));
        wrist = new SimpleServo(hardwareMap, "servo_brat2", 0, 1);
        wrist.setPosition(Range.clip(0.37, 0, 1));

        armRev = hardwareMap.get(DcMotorEx.class, "rev_hd_brat"); armRev.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armRev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_intakeSubsystem = new intakeSubsystem(claw, wristRev, wrist, armRev, telemetry);

        sensorDistanta_intake = hardwareMap.get(DistanceSensor.class, "senzor_gheara");

        leftMotor = hardwareMap.get(DcMotorEx.class, "liftStanga");
        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor = hardwareMap.get(DcMotorEx.class, "liftDreapta");
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        latch= new SimpleServo(hardwareMap, "servo_deposit", 0, 1);
        latch.setPosition(Range.clip(0.1, 0, 1));

        m_depositSubsystem = new depositSubsystem(leftMotor, rightMotor, latch, telemetry);



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(100);





        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {

            wrist.setPosition(Range.clip(0.133, 0, 1));
            wristRev.setPosition(Range.clip(0.41, 0, 1));
            claw.setPosition(Range.clip(0.22, 0, 1));

            latch.setPosition(Range.clip(0.23, 0, 1));


            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }
    }
    /*
     * The START command just came in: now work off the latest snapshot acquired
     * during the init loop.
     */

    @Override
    public void run() {






        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */

        CommandScheduler.getInstance().schedule(
                new preloadTrajectoryLeft(drive),
                new WaitCommand(5000),
                new lowPos(m_intakeSubsystem)
        );


        if (tagOfInterest == null || tagOfInterest.id == MIDDLE) {

        } else if (tagOfInterest.id == LEFT) {

        } else {

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

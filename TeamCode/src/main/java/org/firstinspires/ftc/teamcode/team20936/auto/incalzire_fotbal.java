package org.firstinspires.ftc.teamcode.team20936.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class incalzire_fotbal extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        //drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(-62.17, -62.31, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-62.02, -45.28), Math.toRadians(89.51))
                .splineTo(new Vector2d(-37.50, -39.41), Math.toRadians(-41.67))
                .splineTo(new Vector2d(-32.22, -60.55), Math.toRadians(-32.30))
                .splineTo(new Vector2d(-5.94, -38.24), Math.toRadians(43.07))
                .splineTo(new Vector2d(29.87, -59.08), Math.toRadians(30.57))
                .splineTo(new Vector2d(40.88, -36.33), Math.toRadians(33.97))
                .splineTo(new Vector2d(58.06, -40.59), Math.toRadians(-57.18))
                .splineTo(new Vector2d(54.68, -61.43), Math.toRadians(180.78))
                .splineTo(new Vector2d(-62.46, -62.02), Math.toRadians(180.15))
                .build();
        drive.setPoseEstimate(traj1.start());

        drive.followTrajectorySequence(traj1);

    }

}
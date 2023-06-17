package org.firstinspires.ftc.teamcode.team20936.auto.autoCommands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class preloadTrajectoryLeft extends CommandBase {

    private final SampleMecanumDrive drive;

    public preloadTrajectoryLeft(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(5, 0))
                .lineToLinearHeading(new Pose2d(5,40, Math.toRadians(-45)))
                .build());
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }

}

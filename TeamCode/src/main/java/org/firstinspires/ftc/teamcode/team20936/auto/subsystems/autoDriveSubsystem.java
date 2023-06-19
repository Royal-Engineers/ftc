package org.firstinspires.ftc.teamcode.team20936.auto.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class autoDriveSubsystem extends SubsystemBase {

    private SampleMecanumDrive mec_drive;
    private Trajectory traj;

    public autoDriveSubsystem(HardwareMap hMap) {
        mec_drive = new SampleMecanumDrive(hMap);
    }

    public void drive(Pose2d start, Pose2d end) {
        mec_drive.setPoseEstimate(start);
        traj = mec_drive.trajectoryBuilder(start)

                .lineToLinearHeading(end)

                .build();
        mec_drive.followTrajectory(traj);
    }

}

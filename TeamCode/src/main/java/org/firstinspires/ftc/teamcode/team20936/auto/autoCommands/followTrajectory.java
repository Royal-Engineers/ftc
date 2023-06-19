package org.firstinspires.ftc.teamcode.team20936.auto.autoCommands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.auto.subsystems.autoDriveSubsystem;

public class followTrajectory extends CommandBase {

    autoDriveSubsystem m_autoDriveSubsystem;
    private Pose2d start;
    private Pose2d end;

    public followTrajectory(autoDriveSubsystem subsystem, Pose2d start, Pose2d end) {
        m_autoDriveSubsystem = subsystem;
        this.start = start;
        this.end = end;
        addRequirements(m_autoDriveSubsystem);
    }

    @Override
    public void initialize() { m_autoDriveSubsystem.drive(start, end); }

    @Override
    public boolean isFinished() { return true; }

}

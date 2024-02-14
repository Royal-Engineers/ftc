package org.firstinspires.ftc.teamcode.commands.LiftCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.KeepPosition;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.drive.DriveSubsystem;

public class DriveDistanceSensor extends CommandBase {

    public double DistanceNeededMM = 150d;
    public int FramesNeeded =  5;
    public int Frames = 0;
    public RobotHardware m_RobotHardware;
    public DriveSubsystem m_DriveSubsystem;
    public Telemetry m_Telemetry;

    public DistanceSensor m_DistanceSensor;
    public DriveDistanceSensor(RobotHardware robothardware, DriveSubsystem drive)
    {
        m_RobotHardware = robothardware;
        m_DriveSubsystem = drive;
    }

    @Override
    public void initialize()
    {
        m_DistanceSensor = m_RobotHardware.m_DistanceSensor;
        m_Telemetry = m_RobotHardware.m_telemetry;
        KeepPosition.isActive = false;

    }

    @Override
    public void execute() {
        m_DriveSubsystem.UpdateAuto(0, -0.3, 0);
        if ( m_DistanceSensor.getDistance(DistanceUnit.MM) < DistanceNeededMM)
        Frames++;
    }

    @Override
    public void finalize()
    {
        KeepPosition.isActive = true;
    }
    @Override
    public boolean isFinished()
    {
        return  Frames > FramesNeeded;

    }

}

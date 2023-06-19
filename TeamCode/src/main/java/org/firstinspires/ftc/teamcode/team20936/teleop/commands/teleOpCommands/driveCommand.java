package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.driveSubsystem;

public class driveCommand extends CommandBase {

    driveSubsystem m_driveSubsystem;
    private double strafe;
    private double forward;
    private double rotation;


    public driveCommand(driveSubsystem subsystem, double strafe, double forward, double rotation) {
        m_driveSubsystem = subsystem;
        addRequirements(m_driveSubsystem);
        this.strafe = strafe;
        this.forward = forward;
        this.rotation = rotation;
    }

    @Override
    public void execute() { m_driveSubsystem.drive(strafe, forward, rotation); }



}

package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;
import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.depositSubsystem;

public class transfer extends SequentialCommandGroup {

    public transfer (intakeSubsystem m_intakeSubsystem, depositSubsystem m_depositSubsystem) {
        addCommands(
                new latchOpen(m_depositSubsystem),
                new setArmRevPos(m_intakeSubsystem, -27),
                new setWristRevPos(m_intakeSubsystem, 0.982222),
                new WaitCommand(500),
                new setArmRevPos(m_intakeSubsystem, -50),
                new setWristPos(m_intakeSubsystem,0.6233333),
                new WaitCommand(500),
                new setClawPos(m_intakeSubsystem,0.15),
                new latchClose(m_depositSubsystem),
                new WaitCommand(400),
                new setArmRevPos(m_intakeSubsystem,-50),
                new WaitCommand(250),
                new lowPos(m_intakeSubsystem),
                new clawRelease(m_intakeSubsystem)

        );
        addRequirements(m_intakeSubsystem, m_depositSubsystem);
    }

}

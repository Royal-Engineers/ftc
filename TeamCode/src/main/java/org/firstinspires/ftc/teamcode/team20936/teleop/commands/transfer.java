package org.firstinspires.ftc.teamcode.team20936.teleop.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;
import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.depositSubsystem;
import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.revSubsystem;

public class transfer extends SequentialCommandGroup {

    public transfer (intakeSubsystem m_intakeSubsystem, depositSubsystem m_depositSubsystem, revSubsystem m_revSubsystem) {
        addCommands(
                new latchOpen(m_depositSubsystem),
                new setWristRevPos(m_intakeSubsystem, 0.99),
                new WaitCommand(200),
                new setArmRevPos(m_revSubsystem, -49),
                new setWristPos(m_intakeSubsystem,0.6244444),
                new WaitCommand(450),
                new setClawPos(m_intakeSubsystem,0.15),
                new WaitCommand(70),
                new latchClose(m_depositSubsystem),
                new WaitCommand(300),
                new setArmRevPos(m_revSubsystem,-27),
                new WaitCommand(150),
                new lowPos(m_intakeSubsystem)
        );
        addRequirements(m_intakeSubsystem, m_depositSubsystem);
    }

}


package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.depositSubsystem;
import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;

public class transfer extends SequentialCommandGroup {

    public transfer (intakeSubsystem m_intakeSubsystem, depositSubsystem m_depositSubsystem) {
        addCommands(
                new latchOpen(m_depositSubsystem),
                new setArmRevPos(m_intakeSubsystem, -55),
                new setWristRevPos(m_intakeSubsystem, 0.971),
                new WaitCommand(450),
                new setArmRevPos(m_intakeSubsystem, -44),
                new setWristPos(m_intakeSubsystem,0.678),
                new WaitCommand(450),
                new setClawPos(m_intakeSubsystem,0.15),
                new WaitCommand(100),
                new latchClose(m_depositSubsystem),
                new WaitCommand(200),
                new setArmRevPos(m_intakeSubsystem,-27),
                new WaitCommand(150),

                new setWristPos(m_intakeSubsystem, 0.163),
                new setArmRevPos(m_intakeSubsystem, -50, 0.5),
                new WaitCommand(100),
                new setWristRevPos(m_intakeSubsystem, 0.413),

                new clawRelease(m_intakeSubsystem)

        );

        addRequirements(m_intakeSubsystem, m_depositSubsystem);
    }

}


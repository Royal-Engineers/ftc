
package org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;
import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.depositSubsystem;

public class transfer extends SequentialCommandGroup {

    public transfer (intakeSubsystem m_intakeSubsystem, depositSubsystem m_depositSubsystem) {
        addCommands(
                new latchOpen(m_depositSubsystem),
                new setArmRevPos(m_intakeSubsystem, -55),
                new setWristRevPos(m_intakeSubsystem, 0.971),
                new WaitCommand(500),
                new setArmRevPos(m_intakeSubsystem, -48),
                new setWristPos(m_intakeSubsystem,0.648),
                new WaitCommand(500),
                new setClawPos(m_intakeSubsystem,0.15),
                new latchClose(m_depositSubsystem),
                new WaitCommand(400),
                new setArmRevPos(m_intakeSubsystem,-27),
                new WaitCommand(250),

                new setWristPos(m_intakeSubsystem, 0.163),
                new setArmRevPos(m_intakeSubsystem, -50, 0.5),
                new WaitCommand(100),
                new setWristRevPos(m_intakeSubsystem, 0.413),

                new clawRelease(m_intakeSubsystem)

        );

        addRequirements(m_intakeSubsystem, m_depositSubsystem);
    }

}


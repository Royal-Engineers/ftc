package org.firstinspires.ftc.teamcode.commands.SpecialCommands;

import android.icu.text.Transliterator;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.commands.KeepPosition;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.drive.OdometryComponent;

public class StackCommand extends SequentialCommandGroup {

    public static double IntakeMid = 0.85d, IntakeLow = 0.92d;
    public StackCommand(RobotHardware robot, KeepPosition PositionCommand, double x, double y, double Ang){
        super(
                new InstantCommand(()->{robot.motorIntake.setPower(0.8);}),
                new InstantCommand(()->{robot.m_ServoIntake.setPosition(IntakeMid);}),
                new WaitCommand(1000),
                new FollowPath(x, y ,
                        Ang, PositionCommand),
                new WaitCommand(1000),
                new InstantCommand(()->{robot.m_ServoIntake.setPosition(IntakeLow);}),
                new WaitCommand(1500),
                new Outtake(robot)



                );
    }
}

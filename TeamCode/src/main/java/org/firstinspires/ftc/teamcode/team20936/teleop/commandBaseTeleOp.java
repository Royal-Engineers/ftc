package org.firstinspires.ftc.teamcode.team20936.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.clawRelease;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.driveCommand;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.setArmRevPos;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.setHeight;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.setManualRevPos;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.setManualWristPos;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.setManualWristRevPos;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.stackCommands.setToggleStack;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.stackCommands.stack_2ndcone;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.stackCommands.stack_3rdcone;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.stackCommands.stack_4thcone;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.stackCommands.stack_5thcone;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.toggleSensor;
import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.depositSubsystem;
import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.driveSubsystem;
import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;

import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.lowPos;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.pickUp;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.clawGrab;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.latchClose;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.latchOpen;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.extend;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.retract;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.manualControl;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.teleOpCommands.transfer;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class commandBaseTeleOp extends CommandOpMode{
    private GamepadEx controller1, controller2;

    intakeSubsystem m_intakeSubsystem;
    depositSubsystem m_depositSubsystem;
    driveSubsystem m_driveSubsystem;

    @Override
    public void initialize() {

        telemetry.addData("Status", "Initialized");

        CommandScheduler.getInstance().reset();

        m_intakeSubsystem = new intakeSubsystem(hardwareMap, telemetry);
        m_depositSubsystem = new depositSubsystem(hardwareMap, telemetry);

        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);



        controller2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SequentialCommandGroup(
                        new pickUp(m_intakeSubsystem),
                        new clawRelease(m_intakeSubsystem)
                ));
        controller1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new SequentialCommandGroup(
                        new pickUp(m_intakeSubsystem),
                        new clawRelease(m_intakeSubsystem)
                ));

        controller2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new SequentialCommandGroup(
                        new clawGrab(m_intakeSubsystem),
                        new WaitCommand(200),
                        new lowPos(m_intakeSubsystem)
                ));

        controller2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new transfer(m_intakeSubsystem, m_depositSubsystem));
        controller1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new transfer(m_intakeSubsystem, m_depositSubsystem));

        controller2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(new clawRelease(m_intakeSubsystem), new clawGrab(m_intakeSubsystem));

        controller1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new setHeight(m_depositSubsystem, 1));
        controller1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new setHeight(m_depositSubsystem, 2));

        controller1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new extend(m_depositSubsystem));

        controller1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                        new retract(m_depositSubsystem),
                        new WaitCommand(5),
                        new latchOpen(m_depositSubsystem)
                ));

        controller1.getGamepadButton(GamepadKeys.Button.START)
                .toggleWhenPressed(new latchOpen(m_depositSubsystem), new latchClose(m_depositSubsystem));

        controller2.getGamepadButton(GamepadKeys.Button.START)
                .toggleWhenPressed(new toggleSensor(m_intakeSubsystem));

    }

    @Override
    public void run() {

        CommandScheduler.getInstance().schedule(
                new driveCommand(m_driveSubsystem, controller1.getLeftX(), controller1.getLeftY(), controller1.getRightX())
        );

        if (gamepad1.right_trigger > 0.01 && m_depositSubsystem.getPosition()<=2000)
            CommandScheduler.getInstance().schedule(
                    (new manualControl(m_depositSubsystem, m_depositSubsystem.getPosition() + 80 * (Math.abs((int) gamepad1.right_trigger))))
            );
        else if (gamepad1.left_trigger > 0.01 && m_depositSubsystem.getPosition()>=0) {
            if (m_depositSubsystem.getPosition() > 0)
                CommandScheduler.getInstance().schedule(
                        (new manualControl(m_depositSubsystem, m_depositSubsystem.getPosition() - 80 * (Math.abs((int) gamepad1.left_trigger))))
                );
        }

        if(m_intakeSubsystem.claw.getPosition() > 0.2 && m_intakeSubsystem.coneDetected() && m_intakeSubsystem.getSensorActive()
        && m_intakeSubsystem.armRev.getCurrentPosition() < -480)
        {
            schedule(new SequentialCommandGroup(
                    new clawGrab(m_intakeSubsystem),
                    new WaitCommand(200),
                    new lowPos(m_intakeSubsystem)
            ));
        }



        if(gamepad2.right_stick_y != 0 && m_intakeSubsystem.wrist.getPosition() >= 0 && m_intakeSubsystem.wrist.getPosition() <= 0.7) {
            CommandScheduler.getInstance().schedule(
                    new setManualWristPos(m_intakeSubsystem, gamepad2.right_stick_y)
            );
        }
        if(gamepad2.right_stick_x != 0 && m_intakeSubsystem.wristRev.getPosition() >= 0 && m_intakeSubsystem.wristRev.getPosition() <= 1) {
            CommandScheduler.getInstance().schedule(
                    new setManualWristRevPos(m_intakeSubsystem, gamepad2.right_stick_x)
            );
        }
        if(gamepad2.left_stick_y != 0) {
            CommandScheduler.getInstance().schedule(
                    new setManualRevPos(m_intakeSubsystem, gamepad2.left_stick_y)
            );
        }


        if(m_intakeSubsystem.getToggleStack() == true && m_intakeSubsystem.coneDetected() ) {
            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                    new setArmRevPos(m_intakeSubsystem, -50),
                    new WaitCommand(200),
                    new lowPos(m_intakeSubsystem),
                    new setToggleStack(m_intakeSubsystem, false)
            ));
        }

        controller2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SequentialCommandGroup(
                             new stack_2ndcone(m_intakeSubsystem),
                             new setToggleStack(m_intakeSubsystem, true)
                     ));
        controller2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                        .whenPressed(new SequentialCommandGroup(
                                new stack_3rdcone(m_intakeSubsystem),
                                new setToggleStack(m_intakeSubsystem, true)
                        ));
        controller2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                        .whenPressed(new SequentialCommandGroup(
                                new stack_4thcone(m_intakeSubsystem),
                                new setToggleStack(m_intakeSubsystem, true)
                        ));
        controller2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                        .whenPressed(new SequentialCommandGroup(
                                new stack_5thcone(m_intakeSubsystem),
                                new setToggleStack(m_intakeSubsystem,true)
                        ));


        CommandScheduler.getInstance().run();
        telemetry.update();

        }

}

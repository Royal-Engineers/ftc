package org.firstinspires.ftc.teamcode.team20936.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.depositSubsystem;
import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;

import org.firstinspires.ftc.teamcode.team20936.teleop.commands.lowPos;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.pickUp;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.clawGrab;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.latchClose;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.latchOpen;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.raiseHigh;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.retract;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.raiseMiddle;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.manualControl;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.setManualArmRevPos;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.setManualWristPos;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.setManualWristRevPos;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.setClawPos;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.transfer;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class commandBaseTeleOp extends CommandOpMode{

    private boolean claw_toggle = true;
    private boolean latch_toggle = true;
    private boolean sens_toggle = true;
    private Motor fL, fR, bR, bL;
    private DcMotor armRev;
    private DcMotorEx leftMotor, rightMotor;
    private MecanumDrive m_drive;
    private ServoEx claw, wristRev, wrist, latch;
    private GamepadEx controller1, controller2;
    private DistanceSensor sensorDistanta_intake;
    private int slideHeight;

    intakeSubsystem m_intakeSubsystem;
    depositSubsystem m_depositSubsystem;

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();

        telemetry.addData("Status", "Initialized");

        claw = new SimpleServo(hardwareMap, "servo_gheara", 0, 1);
        claw.setPosition(Range.clip(0.22, 0, 1));
        wristRev = new SimpleServo(hardwareMap, "servo_brat3", 0, 1);
        wristRev.setPosition(Range.clip(0.4, 0, 1));
        wrist = new SimpleServo(hardwareMap, "servo_brat2", 0, 1);
        wrist.setPosition(Range.clip(0.37, 0, 1));
        armRev = hardwareMap.dcMotor.get("rev_hd_brat");
        armRev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_intakeSubsystem = new intakeSubsystem(claw, wristRev, wrist, armRev, telemetry);

        sensorDistanta_intake = hardwareMap.get(DistanceSensor.class, "senzor_gheara");

        fR = new Motor(hardwareMap, "motorFrontRight", Motor.GoBILDA.RPM_312); fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fL = new Motor(hardwareMap, "motorFrontLeft", Motor.GoBILDA.RPM_312); fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR = new Motor(hardwareMap, "motorBackRight", Motor.GoBILDA.RPM_312); bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL = new Motor(hardwareMap, "motorBackLeft", Motor.GoBILDA.RPM_312); bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_drive = new MecanumDrive(fL, fR, bL, bR);

        leftMotor = hardwareMap.get(DcMotorEx.class, "liftStanga"); leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightMotor = hardwareMap.get(DcMotorEx.class, "liftDreapta"); rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        latch= new SimpleServo(hardwareMap, "servo_deposit", 0, 1); latch.setPosition(Range.clip(0.1, 0, 1));
        m_depositSubsystem = new depositSubsystem(leftMotor, rightMotor, latch, telemetry);

        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

        slideHeight = 1;


    }

    @Override
    public void run() {


        m_drive.driveRobotCentric(-controller1.getLeftX(), -controller1.getLeftY(), -controller1.getRightX());



        if (gamepad2.right_bumper)
        {
            claw_toggle = !claw_toggle;
            if (claw_toggle) { claw.setPosition(Range.clip(0.22, 0, 1)); }
            else { claw.setPosition(Range.clip(0.0, 0, 1));}

            sleep(100);
        }

        if(gamepad2.a){
            CommandScheduler.getInstance().schedule(
              new pickUp(m_intakeSubsystem)
            );
        }

        if(gamepad2.b) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new clawGrab(m_intakeSubsystem),
                            new WaitCommand(100),
                            new lowPos(m_intakeSubsystem)
                    )
            );
        }

        if (gamepad2.options) {
            sens_toggle = ! sens_toggle;
            sleep(100);
        }

        if(claw.getPosition() > 0.2 && sensorDistanta_intake.getDistance(DistanceUnit.MM) < 30 && sens_toggle == true)
        {
            CommandScheduler.getInstance().schedule(
                    new clawGrab(m_intakeSubsystem),
                    new WaitCommand(100),
                    new lowPos(m_intakeSubsystem)
            );
        }

        if (gamepad1.a) {
            slideHeight = 1;
        } else if (gamepad1.b) {
            slideHeight = 2;
        }

        if (gamepad1.left_bumper)
            switch (slideHeight){
                case 1: {
                    CommandScheduler.getInstance().schedule(
                            new raiseMiddle(m_depositSubsystem)
                    );
                    break;
                }
                case 2: {
                    CommandScheduler.getInstance().schedule(
                            new raiseHigh(m_depositSubsystem)
                    );
                    break;
                }
            }
        else if (gamepad1.right_bumper) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new retract(m_depositSubsystem),
                            new WaitCommand(10),
                            new latchOpen(m_depositSubsystem)
                    )
            );
        }

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

        if (gamepad1.options) {
            if (latch_toggle) {
                CommandScheduler.getInstance().schedule(new latchOpen(m_depositSubsystem));
                latch_toggle = !latch_toggle;

                sleep(100);
            }
            else {
                CommandScheduler.getInstance().schedule(new latchClose(m_depositSubsystem));
                latch_toggle = !latch_toggle;

                sleep(100);
            }
        }

        CommandScheduler.getInstance().schedule(
                new setManualArmRevPos(m_intakeSubsystem, gamepad2.left_stick_y)
        );
        CommandScheduler.getInstance().schedule(
                new setManualWristPos(m_intakeSubsystem, gamepad2.right_stick_y)
        );
        CommandScheduler.getInstance().schedule(
                new setManualWristRevPos(m_intakeSubsystem, gamepad2.right_stick_x)
        );

        if(gamepad2.y) {
            CommandScheduler.getInstance().schedule(
                    new transfer(m_intakeSubsystem, m_depositSubsystem)
            );
        }



        CommandScheduler.getInstance().run();
        telemetry.update();


        }

}

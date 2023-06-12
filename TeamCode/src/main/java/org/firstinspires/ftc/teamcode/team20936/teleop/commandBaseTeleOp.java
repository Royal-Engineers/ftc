package org.firstinspires.ftc.teamcode.team20936.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.lowPos;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.pickUp;
import org.firstinspires.ftc.teamcode.team20936.teleop.subsystems.intakeSubsystem;

import org.firstinspires.ftc.teamcode.team20936.teleop.commands.clawRelease;
import org.firstinspires.ftc.teamcode.team20936.teleop.commands.clawGrab;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.TimeUnit;

@TeleOp
public class commandBaseTeleOp extends CommandOpMode{

    private boolean claw_toggle = true;

    private Motor fL, fR, bR, bL;
    private DcMotor armRev;
    private MecanumDrive m_drive;
    private ServoEx claw, wristRev, wrist;
    private GamepadEx controller1, controller2;
    private DistanceSensor sensorDistanta_intake;

    intakeSubsystem m_intakeSubsystem;
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

        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);


    }

    @Override
    public void run() {

        m_drive.driveRobotCentric(-controller1.getLeftX(), -controller1.getLeftY(), -controller1.getRightX());

        if (gamepad2.right_bumper)
        {
            claw_toggle = !claw_toggle;
            if (claw_toggle) { claw.setPosition(Range.clip(0.22, 0, 1)); }
            else { claw.setPosition(Range.clip(0.0, 0, 1));} // DESCHIS ( SE EXECUTA PRIMA)

            sleep(100); // de optimizat !!
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

        if(claw.getPosition() > 0.2 && sensorDistanta_intake.getDistance(DistanceUnit.MM) < 30)
        {
            CommandScheduler.getInstance().schedule(
                    new clawGrab(m_intakeSubsystem),
                    new WaitCommand(100),
                    new lowPos(m_intakeSubsystem)
            );
        }


        CommandScheduler.getInstance().run();

            telemetry.update();

        }


}

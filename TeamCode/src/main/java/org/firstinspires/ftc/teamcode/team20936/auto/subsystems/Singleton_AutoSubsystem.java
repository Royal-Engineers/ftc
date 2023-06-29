package org.firstinspires.ftc.teamcode.team20936.auto.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.ServoEx;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.team20936.auto.subsystems.Lift;

import org.apache.commons.math3.analysis.function.Sin;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.Range;


public class Singleton_AutoSubsystem extends SubsystemBase{

    private static volatile Singleton_AutoSubsystem instance = null;
    private  ServoEx m_wrist;
    private  ServoEx m_wristRev;
    private  ServoEx m_claw;
    private  ServoEx m_latch;
    public  SampleMecanumDrive m_drive;
    private  DcMotor m_armRev;
    private  Lift m_lift;

    public TrajectorySequence m_trajectory;


    public int poz0_rev;

    public static double clawGrabPos = 0.00;
    public static double clawReleasePos = 0.22;
    public static double clawMedianPos = 0.15;
    double MIN_POSITION = 0;
    double MAX_POSITION = 1;
    private Singleton_AutoSubsystem(ServoEx wrist, ServoEx wristRev, ServoEx claw, ServoEx latch,
                                   SampleMecanumDrive drive, DcMotor armRev, Lift lift)
    {
        m_wrist = wrist;
        m_wristRev = wristRev;
        m_claw = claw;
        m_latch = latch;
        m_drive = drive;
        m_armRev = armRev;
        m_lift = lift;
        poz0_rev = armRev.getCurrentPosition();
    }

    static public Singleton_AutoSubsystem createInstance(ServoEx wrist, ServoEx wristRev, ServoEx claw, ServoEx latch,
                                               SampleMecanumDrive drive, DcMotor armRev, Lift lift)
    {

        synchronized (Singleton_AutoSubsystem.class)
        {
            assert ( instance == null );
            instance = new Singleton_AutoSubsystem(wrist,  wristRev,  claw,  latch, drive,  armRev,  lift);
        }
        return instance;
    }

    static public Singleton_AutoSubsystem getInstance() {
        assert(instance != null );
        return instance;
    }
    public Singleton_AutoSubsystem setClawPos(double target)
    {
       m_claw.setPosition(Range.clip(target, MIN_POSITION, MAX_POSITION));
       return this;
    }

    public Singleton_AutoSubsystem setWristPos(double target)
    {
        m_wrist.setPosition(Range.clip(target, MIN_POSITION, MAX_POSITION));
        return this;
    }

    public Singleton_AutoSubsystem setWristRevPos(double target)
    {
        m_wristRev.setPosition(Range.clip(target, MIN_POSITION, MAX_POSITION));
        return this;
    }

    public Singleton_AutoSubsystem setLatchPos(double target)
    {
        m_latch.setPosition(Range.clip(target, MIN_POSITION, MAX_POSITION));
        return this;
    }

    public Singleton_AutoSubsystem setArmRevPos(int target, double power)
    {
        m_armRev.setTargetPosition(poz0_rev+target);
        m_armRev.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        m_armRev.setPower(power);
        return this;
    }

    public Singleton_AutoSubsystem setLiftPos(int target)
    {
        m_lift.setLiftPosition(target);
        return this;
    }

    public Singleton_AutoSubsystem gotoConstantHeading(double targetX, double targetY)
    {

        m_trajectory = m_drive.trajectorySequenceBuilder(m_drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(targetX, targetY))
                .build();
        m_drive.followTrajectorySequence(m_trajectory);
        return this;
    }

    public Singleton_AutoSubsystem gotoLinearHeading(double targetX, double targetY, double TargetHeading_Degrees)
    {
        m_trajectory = m_drive.trajectorySequenceBuilder(m_drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(targetX, targetY, Math.toRadians(TargetHeading_Degrees)))
                .build();
        m_drive.followTrajectorySequence(m_trajectory);

        return this;
    }


}

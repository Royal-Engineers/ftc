package org.firstinspires.ftc.teamcode.facade.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;


public class BombasticLift {

    private static final int s_DefaultTolerance = 10;

    private static final double s_MinPower = 0.1d;

    private static final double s_DefaultPower = -1.0d;

    private static final double s_SafetyPower = -0.5d;
    public double m_MinPosition = 0.0d, m_MaxPosition = 0.0d ;

    private double m_TargetPosition = 0.0d;

    private RobotHardware m_Robot;
    private Telemetry m_Telemetry;
    //initialized motors
    private DcMotorEx m_MainMotor, m_InvMotor;

    //pe viitor refa cu Hashuri
    public enum e_LiftPosition
    {
        LowPos, MidPos, MaxPos
    }

    public BombasticLift(RobotHardware robot, Telemetry telemetry, DcMotorEx MainMotor, DcMotorEx InvMotor,
                         double MinPositon, double MaxPosition)
    {
        m_Robot = robot;
        m_Telemetry = telemetry;
        m_MainMotor = MainMotor;
        m_InvMotor = InvMotor;
        m_MaxPosition = MaxPosition;
        m_MinPosition = MinPositon;
    }

    public void SetRawPower(double power)
    {
        if ( Math.abs(power) > s_MinPower ) {
            m_MainMotor.setPower(power);
            m_InvMotor.setPower(-power);
        }
        else {
            m_MainMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m_InvMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
    }
    public int GetLiftPosition()
    {
        return Math.abs(m_InvMotor.getCurrentPosition());
    }
    public void IncrementTargetPosition(double inc) {
        FollowTarget(m_TargetPosition += inc, s_DefaultPower);
    }

    public void SetTargetPosition(double target, double power) {
        FollowTarget(target, power);
    }

    public void SetStatePosition(e_LiftPosition position)
    {
        switch(position)
        {
            case LowPos:
                SetTargetPosition(m_MinPosition, s_SafetyPower);
                break;
            case MaxPos:
                SetTargetPosition(m_MaxPosition, s_DefaultPower);
                break;
            case MidPos:
                SetTargetPosition( (m_MaxPosition + m_MinPosition) / 2, s_DefaultPower);
                break;
            default:
                SetTargetPosition( 0, s_SafetyPower );
                break;
        }
    }

    public void FollowTarget(double target, double power)
    {
        if ( target < m_MinPosition - s_DefaultTolerance || target > m_MaxPosition + s_DefaultTolerance)
            return;
        m_TargetPosition = target;
        m_MainMotor.setTargetPositionTolerance(s_DefaultTolerance);
        m_InvMotor.setTargetPositionTolerance(s_DefaultTolerance);

        if ( GetLiftPosition() > m_TargetPosition)
            power = -power;

        m_MainMotor.setTargetPosition((int) -m_TargetPosition);
        m_InvMotor.setTargetPosition((int) m_TargetPosition);

        m_MainMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        m_InvMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        m_MainMotor.setPower(power);
        m_InvMotor.setPower(-power);

    }

    public boolean IsWithinTolerance()
    {
        return Math.abs(m_MainMotor.getCurrentPosition() - m_MainMotor.getTargetPosition())
                < m_MainMotor.getTargetPositionTolerance();
    }
    public void UpdateTelemetry()
    {
        if( RobotHardware.DebugMode == false )
            return;
        m_Telemetry.addData("Lift Telemetry -> ", "BEGIN");
        m_Telemetry.addData("InvLiftPos", m_InvMotor.getCurrentPosition());
        m_Telemetry.addData("MainLiftPos", m_MainMotor.getCurrentPosition());
        m_Telemetry.addData("Lift Telemetry -> ", "END");
    }
}

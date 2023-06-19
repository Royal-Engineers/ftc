package org.firstinspires.ftc.teamcode.team20936.teleop.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class driveSubsystem extends SubsystemBase {

    private Motor fL, fR, bR, bL;
    private MecanumDrive m_drive;

    public driveSubsystem(HardwareMap hMap, Telemetry telemetry) {

        fR = new Motor(hMap, "motorFrontRight", Motor.GoBILDA.RPM_312); fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fL = new Motor(hMap, "motorFrontLeft", Motor.GoBILDA.RPM_312); fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR = new Motor(hMap, "motorBackRight", Motor.GoBILDA.RPM_312); bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL = new Motor(hMap, "motorBackLeft", Motor.GoBILDA.RPM_312); bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        m_drive = new MecanumDrive(fL, fR, bL, bR);
    }

    public void drive(double strafe, double forward, double rotation) {
       m_drive.driveRobotCentric(-strafe, -forward, -rotation);
    }


}

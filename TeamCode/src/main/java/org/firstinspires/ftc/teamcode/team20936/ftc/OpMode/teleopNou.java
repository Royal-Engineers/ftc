package org.firstinspires.ftc.teamcode.team20936.ftc.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team20936.OpenCv.ConeDetection;
import org.firstinspires.ftc.teamcode.team20936.OpenCv.GetFrameImage;
import org.firstinspires.ftc.teamcode.team20936.Utils.globals;
import org.firstinspires.ftc.teamcode.team20936.ftc.CameraUtil;
import org.firstinspires.ftc.teamcode.team20936.ftc.Robot;
import org.opencv.core.Mat;

import java.util.Calendar;

@TeleOp
@Config
public class teleopNou extends CommandOpMode {


    private CameraUtil m_CameraUtil;
    private Robot m_robot;


    @Override public void initialize()
    {
        m_robot = new Robot(telemetry, hardwareMap);
        m_CameraUtil = new CameraUtil();
    }

    @Override public void run()
    {


        m_CameraUtil.update();


    }
}

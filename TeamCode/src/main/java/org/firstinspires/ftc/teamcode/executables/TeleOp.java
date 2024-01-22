package org.firstinspires.ftc.teamcode.executables;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.interfaces.i_gamepad;
import org.firstinspires.ftc.teamcode.facade.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.facade.drive.OdometryComponent;
import org.firstinspires.ftc.teamcode.facade.controllers.controller1;
import org.firstinspires.ftc.teamcode.facade.controllers.controller2;
import org.firstinspires.ftc.teamcode.pipelines.PipelineDreapta;
import org.openftc.easyopencv.OpenCvPipeline;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode {

    private RobotHardware Robot;

    private DriveSubsystem m_DriveSubsystem;


    private OdometryComponent m_odometry;

    private OpenCvPipeline pipeline;

    private i_gamepad m_controller1, m_controller2;
    @Override
    public void initialize() {
        super.reset();




        Robot = new RobotHardware();
        Robot.init(gamepad1, gamepad2, telemetry, hardwareMap);

        pipeline = new PipelineDreapta(telemetry, PipelineDreapta.team.rosu);
        Robot.camera.setPipeline(pipeline);

        m_DriveSubsystem = new DriveSubsystem(Robot);

        m_odometry = new OdometryComponent(Robot);

        m_controller1 = new controller1(gamepad1, Robot);
        m_controller2 = new controller2(gamepad2, Robot);

        m_controller1.initialize();
        m_controller2.initialize();
        waitForStart();
        Robot.camera.setPipeline(null);


    }

    @Override
    public void run(){

        Robot.camera.setPipeline(null);
        m_DriveSubsystem.UpdateGamepad();
        m_odometry.update();
        m_controller1.update();
        m_controller2.update();
        Robot.Update();
        telemetry.update();

        super.run();

    }


}

package org.firstinspires.ftc.teamcode.executables;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.interfaces.i_gamepad;
import org.firstinspires.ftc.teamcode.facade.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.facade.subsystems.OdometryComponent;
import org.firstinspires.ftc.teamcode.facade.subsystems.controllers.controller1;
import org.firstinspires.ftc.teamcode.facade.subsystems.controllers.controller2;
import org.firstinspires.ftc.teamcode.pipelines.Pipeline;
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


        Robot = RobotHardware.getInstance();
        Robot.init(gamepad1, gamepad2, telemetry, hardwareMap);

        pipeline = new Pipeline(telemetry);
        Robot.camera.setPipeline(pipeline);

        m_DriveSubsystem = new DriveSubsystem(Robot);

        m_odometry = new OdometryComponent(Robot);

        m_controller1 = new controller1(gamepad1, Robot);
        m_controller2 = new controller2(gamepad2, Robot);
        waitForStart();
    }

    @Override
    public void run(){
        m_DriveSubsystem.UpdateGamepad();
        m_odometry.update();
        m_controller1.update();
        m_controller2.update();
        telemetry.update();

        super.run();

    }


}

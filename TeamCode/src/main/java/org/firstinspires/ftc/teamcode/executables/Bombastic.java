package org.firstinspires.ftc.teamcode.executables;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.GotoX;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.interfaces.i_gamepad;
import org.firstinspires.ftc.teamcode.facade.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.facade.drive.OdometryComponent;
import org.openftc.easyopencv.OpenCvPipeline;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class Bombastic extends CommandOpMode {

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

        //pipeline = new Pipeline(telemetry);
        // Robot.camera.setPipeline(pipeline);

        m_DriveSubsystem = new DriveSubsystem(Robot);

        m_odometry = new OdometryComponent(Robot);

        m_controller1.initialize();
        waitForStart();
    }

    boolean ok = false;
    @Override
    public void run() {

        m_odometry.update();
        if (!ok)
        {
            CommandScheduler.getInstance().schedule(new GotoX(10.0d, telemetry, m_DriveSubsystem));
        ok = true;
        }
        telemetry.update();

        super.run();

    }


}

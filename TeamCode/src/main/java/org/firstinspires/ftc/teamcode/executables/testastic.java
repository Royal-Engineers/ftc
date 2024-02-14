package org.firstinspires.ftc.teamcode.executables;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.GotoTheta;
import org.firstinspires.ftc.teamcode.commands.GotoX;
import org.firstinspires.ftc.teamcode.commands.GotoY;
import org.firstinspires.ftc.teamcode.commands.KeepPosition;
import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.interfaces.i_gamepad;
import org.firstinspires.ftc.teamcode.facade.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.facade.drive.OdometryComponent;
import org.firstinspires.ftc.teamcode.pipelines.PipelineStanga;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Autonomous
public class testastic extends CommandOpMode {

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

        pipeline = new PipelineStanga(telemetry,  PipelineStanga.team.albastru);
        Robot.camera.setPipeline(pipeline);

        m_DriveSubsystem = new DriveSubsystem(Robot);

        m_odometry = new OdometryComponent(Robot);

        //m_controller1.initialize();

        waitForStart();
        Robot.camera.setPipeline(null);


    }

    boolean ok = false;

    GotoX XCommand;
    GotoY YCommand;
    GotoTheta TCommand;

    KeepPosition PositionCommand;
    public static double IncrementLeft = 12.5d;
    public static double IncrementRight = -13.0d;
    public static double zone = 2;

    public static double x1 =0.0d, y1 =0.0d, t1 =0.0d;
    public static double x2 =0.0d, y2 =0.0d, t2 =0.0d;

    public static double x3 =0.0d, y3 =0.0d, t3 =0.0d;

    @Override
    public void run() {
        m_odometry.update();
        PipelineStanga.regions Zone = PipelineStanga.region_of_interest;

        m_odometry.update();

        if (!ok)
        {

            XCommand = new GotoX(0.0d, telemetry, m_DriveSubsystem, Robot);
            YCommand = new GotoY(0.0d, telemetry, m_DriveSubsystem, Robot);
            TCommand = new GotoTheta(0.0d, telemetry, m_DriveSubsystem, Robot);

            PositionCommand = new KeepPosition(XCommand, YCommand, TCommand, m_DriveSubsystem);

            CommandScheduler.getInstance().schedule(PositionCommand);

                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                        new FollowPath(x1, y1, t1, PositionCommand, 10, 10, 10),
                        new FollowPath(x2, y2, t2, PositionCommand, 10, 10, 10),
                        new FollowPath(x3, y3, t3, PositionCommand)


                        ));




            ok = true;
        }
        telemetry.addData("ZONAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA", Zone);

        telemetry.update();
        Robot.Update();
        super.run();

    }


}

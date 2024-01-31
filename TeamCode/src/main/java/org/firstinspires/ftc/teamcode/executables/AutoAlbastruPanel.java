package org.firstinspires.ftc.teamcode.executables;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.GotoTheta;
import org.firstinspires.ftc.teamcode.commands.GotoX;
import org.firstinspires.ftc.teamcode.commands.GotoY;
import org.firstinspires.ftc.teamcode.commands.KeepPosition;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftMiddle;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.Retract;
import org.firstinspires.ftc.teamcode.commands.MergiBa;
import org.firstinspires.ftc.teamcode.commands.Transfer;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.interfaces.i_gamepad;
import org.firstinspires.ftc.teamcode.facade.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.facade.drive.OdometryComponent;
import org.firstinspires.ftc.teamcode.pipelines.PipelineStanga;
import org.openftc.easyopencv.OpenCvPipeline;

@Disabled
@Config
@Autonomous
public class AutoAlbastruPanel extends CommandOpMode {

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

    public static double IncrementLeft = -9.5d;
    public static double IncrementRight = 16.0d;
    public static PipelineStanga.regions zone = PipelineStanga.regions.middle;

    @Override
    public void run() {
        m_odometry.update();
         zone = PipelineStanga.region_of_interest;

        double increment = 3;
        if ( zone == PipelineStanga.regions.left)
            increment  = IncrementLeft;
        else if ( zone == PipelineStanga.regions.right )
            increment = IncrementRight;

        if (!ok)
        {
            XCommand = new GotoX(0.0d, telemetry, m_DriveSubsystem, Robot);
             YCommand = new GotoY(0.0d, telemetry, m_DriveSubsystem, Robot);
             TCommand = new GotoTheta(0.0d, telemetry, m_DriveSubsystem, Robot);

             PositionCommand = new KeepPosition(XCommand, YCommand, TCommand, m_DriveSubsystem);

            CommandScheduler.getInstance().schedule(PositionCommand);

                    CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                            new Transfer(Robot).alongWith(
                            new MergiBa(52+ increment, 60, 90, PositionCommand)),
                            new LiftMiddle(Robot).alongWith(
                            new MergiBa(52+ increment, 83., 90, PositionCommand, 0.9, 0.9, 0.9)),
                            new WaitCommand(900),
                            new Retract(Robot),
                            new MergiBa(5, 75.5, 90, PositionCommand,0.9, 0.9, 0.9),
                            new InstantCommand(()->
                            {
                                Robot.m_Lift.SetTargetPosition(0, 0.9);
                            })


                            // new InstantCommand(()->{PositionCommand.SetTheta(   180.0d);})

                            )
        );

            ok = true;
        }
        telemetry.update();
        Robot.Update();
        super.run();

    }


}

package org.firstinspires.ftc.teamcode.executables;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
import org.firstinspires.ftc.teamcode.pipelines.PipelineDreapta;
import org.firstinspires.ftc.teamcode.pipelines.PipelineStanga;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Autonomous
public class auto1 extends CommandOpMode {

    public static double zone1_1t = -90, zone1_1x = 50, zone1_1y = -104.5,
                         zone1_2t = -90, zone1_2x = 70, zone1_2y = -63,
                         zone1_3t = -90, zone1_3x = 5, zone1_3y = -90,


            zone2_1t = -90, zone2_1x = 65, zone2_1y = -104.5,
            zone2_2t = -90, zone2_2x = 91.5, zone2_2y = -36,
            zone2_3t = -90, zone2_3x = 5, zone2_3y = -90,


                         zone3_1t = -90, zone3_1x = 80, zone3_1y = -104.5,
                                 zone3_2t = -90, zone3_2x = 65, zone3_2y = -1,
                        zone3_3t = -90, zone3_3x = 5, zone3_3y = -90;

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

        pipeline = new PipelineStanga(telemetry,  PipelineStanga.team.rosu);
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

            if ( PipelineStanga.region_of_interest == PipelineStanga.regions.left)
                zone = 3;
            else if ( PipelineStanga.region_of_interest == PipelineStanga.regions.middle)
                zone = 2;
            else
                zone = 1;

            if(zone == 1) {

                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                        new ParallelCommandGroup(

                                new Transfer(Robot),
                        new MergiBa(zone1_1x, zone1_1y, zone1_1t, PositionCommand)),
                        new LiftMiddle(Robot),
                        new WaitCommand(700),
                        new ParallelCommandGroup(
                                new Retract(Robot),
                        new MergiBa(zone1_2x, zone1_2y, zone1_2t, PositionCommand)),

                        new InstantCommand(() -> {
                            Robot.m_ServoIntake.setPosition(0.7);
                        }),
                        new WaitCommand(500),
                        new MergiBa(zone1_3x, zone1_3y, zone1_3t, PositionCommand)

                ));

            }

            else if(zone == 2) {

                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                        new ParallelCommandGroup(

                                new Transfer(Robot),
                        new MergiBa(zone2_1x, zone2_1y, zone2_1t, PositionCommand)),
                        new LiftMiddle(Robot),
                        new WaitCommand(700),
                        new ParallelCommandGroup(
                                new Retract(Robot),
                        new MergiBa(zone2_2x, zone2_2y, zone2_2t, PositionCommand)),

                        new InstantCommand(() -> {
                            Robot.m_ServoIntake.setPosition(0.7);
                        }),
                        new WaitCommand(500),

                        new MergiBa(zone2_3x, zone2_3y, zone2_3t, PositionCommand)

                ));

            }

            else if(zone == 3) {

                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new Transfer(Robot),
                        new MergiBa(zone3_1x, zone3_1y, zone3_1t, PositionCommand)),
                        new LiftMiddle(Robot),
                        new WaitCommand(700),
                        new ParallelCommandGroup(
                                new Retract(Robot),
                        new MergiBa(zone3_2x, zone3_2y, zone3_2t, PositionCommand)),


                        new InstantCommand(() -> {
                            Robot.m_ServoIntake.setPosition(0.7);
                        }),
                        new WaitCommand(500),
                        new MergiBa(zone3_3x, zone3_3y, zone3_3t, PositionCommand)

                ));

            }


            ok = true;
        }
        telemetry.addData("ZONAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA", Zone);

        telemetry.update();
        Robot.Update();
        super.run();

    }


}

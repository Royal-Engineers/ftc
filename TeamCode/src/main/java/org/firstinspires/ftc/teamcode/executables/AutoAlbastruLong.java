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
import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.commands.Transfer;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.interfaces.i_gamepad;
import org.firstinspires.ftc.teamcode.facade.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.facade.drive.OdometryComponent;
import org.firstinspires.ftc.teamcode.pipelines.PipelineStanga;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Autonomous
public class AutoAlbastruLong extends CommandOpMode {



    public double pos1t, pos1x, pos1y;
    public double pos2t, pos2x, pos2y;

    public double pos3t, pos3x, pos3y;

    public double pos4t, pos4x, pos4y;

    public double pos5t, pos5x, pos5y;

    public static double zone1_1t = 90, zone1_1x = 30, zone1_1y = -20,
            zone1_2t = 90, zone1_2x =30, zone1_2y = -20,
            zone1_3t = 90, zone1_3x = 30, zone1_3y = -20,


    zone2_1t = 90, zone2_1x = 30, zone2_1y = -20,
            zone2_2t = 90, zone2_2x = 30, zone2_2y = -20,
            zone2_3t = 90, zone2_3x = 30, zone2_3y = -20,


    zone3_1t = 90, zone3_1x = 30, zone3_1y = -20,
            zone3_2t = 90, zone3_2x = 30, zone3_2y = -20,
            zone3_3t = 90, zone3_3x = 30, zone3_3y = -20;

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
                zone = 1;
            else if ( PipelineStanga.region_of_interest == PipelineStanga.regions.middle)
                zone = 2;
            else
                zone = 3;

            if(zone == 1) {

                pos1t = zone1_1t; pos1x = zone1_1x; pos1y = zone1_1y;
                pos2t = zone1_2t; pos2x = zone1_2x; pos2y = zone1_2y;
                pos3t = zone1_3t; pos3x = zone1_3x; pos3y = zone1_3y;
            }

            else if(zone == 2) {

                pos1t = zone2_1t; pos1x = zone2_1x; pos1y = zone2_1y;
                pos2t = zone2_2t; pos2x = zone2_2x; pos2y = zone2_2y;
                pos3t = zone2_3t; pos3x = zone2_3x; pos3y = zone2_3y;

            }

            else if(zone == 3) {
                pos1t = zone3_1t; pos1x = zone3_1x; pos1y = zone3_1y;
                pos2t = zone3_2t; pos2x = zone3_2x; pos2y = zone3_2y;
                pos3t = zone3_3t; pos3x = zone3_3x; pos3y = zone3_3y;


            }

            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new FollowPath(pos1x, pos1y, pos1t, PositionCommand, 10, 10, 10)),
                            new FollowPath(pos2x, pos2y, pos2t, PositionCommand),
                    new InstantCommand(() -> {
                        Robot.m_ServoIntake.setPosition(0.7);
                    }),
                    new WaitCommand(500),
                    new FollowPath(pos3x, pos3y, pos3t, PositionCommand)

            ));
            ok = true;
        }
        telemetry.addData("ZONAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA", Zone);

        telemetry.update();
        Robot.Update();
        super.run();

    }


}

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
import org.firstinspires.ftc.teamcode.commands.SpecialCommands.StackCommand;
import org.firstinspires.ftc.teamcode.commands.Transfer;
import org.firstinspires.ftc.teamcode.facade.RobotHardware;
import org.firstinspires.ftc.teamcode.facade.interfaces.i_gamepad;
import org.firstinspires.ftc.teamcode.facade.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.facade.drive.OdometryComponent;
import org.firstinspires.ftc.teamcode.pipelines.PipelineStanga;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Autonomous
public class auto0nu extends CommandOpMode {



    public static double zone1_1t = 90, zone1_1x = 41, zone1_1y = 84,
            zone1_2t = 90, zone1_2x = 72, zone1_2y = 34,
            zone1_3t = 90, zone1_3x = 2, zone1_3y = 65,


    zone2_1t = 90, zone2_1x = 56, zone2_1y = 84,
            zone2_2t = 90, zone2_2x = 90, zone2_2y = 10,
            zone2_3t = 90, zone2_3x = 2, zone2_3y = 65,


    zone3_1t = 90, zone3_1x = 80, zone3_1y = 84,
            zone3_2t = 90, zone3_2x = 68, zone3_2y = -19,
            zone3_3t = 90, zone3_3x = 120, zone3_3y = 40,
            common_1t = 90, common_1x = 157, common_1y = 20,
            common_2t = 90, common_2x = 157, common_2y = -150,

    common_3t = 90, common_3x = 123, common_3y = -150,
            common_4t = 90, common_4x = 123, common_4y = -188.5,
            common_5t = 90, common_5x = 145, common_5y = -150,
            common_6t = 90, common_6x = 145, common_6y = 20;







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
    SequentialCommandGroup DeFacut;
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

                DeFacut = new SequentialCommandGroup(
                        new ParallelCommandGroup(

                                new Transfer(Robot),

                                new FollowPath(zone1_1x, zone1_1y - 10, zone1_1t, PositionCommand)
                        ),
                        new FollowPath(zone1_1x, zone1_1y, zone1_1t, PositionCommand),

                        new LiftMiddle(Robot),
                        new WaitCommand(700),
                        new ParallelCommandGroup(
                                new Retract(Robot),
                                new FollowPath(zone1_2x, zone1_2y, zone1_2t, PositionCommand)),
                        new InstantCommand(() -> {
                            Robot.m_ServoIntake.setPosition(0.7);
                        }),
                        new WaitCommand(500)//,
                        // new FollowPath(zone1_3x, zone1_3y, zone1_3t, PositionCommand)

                );

            }

            else if(zone == 2) {

                DeFacut = new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new Transfer(Robot),
                                new FollowPath(zone2_1x, zone2_1y - 10, zone2_1t, PositionCommand)),
                        new FollowPath(zone2_1x, zone2_1y, zone2_1t, PositionCommand),


                        new LiftMiddle(Robot),
                        new WaitCommand(700),
                        new ParallelCommandGroup(
                                new Retract(Robot),
                                new FollowPath(zone2_2x, zone2_2y, zone2_2t, PositionCommand)),
                        new InstantCommand(() -> {
                            Robot.m_ServoIntake.setPosition(0.7);
                        }),
                        new WaitCommand(500)//,
                        // new FollowPath(zone2_3x, zone2_3y, zone2_3t, PositionCommand)

                );

            }

            else
            if(zone == 3) {

                DeFacut = new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new Transfer(Robot),
                                new FollowPath(zone3_1x, zone3_1y - 10, zone3_1t, PositionCommand)),
                        new FollowPath(zone3_1x, zone3_1y, zone3_1t, PositionCommand),

                        new LiftMiddle(Robot),
                        new WaitCommand(700),
                        new ParallelCommandGroup(
                                new Retract(Robot),

                                new FollowPath(zone3_2x, zone3_2y, zone3_2t, PositionCommand)),
                        new InstantCommand(() -> {
                            Robot.m_ServoIntake.setPosition(0.7);
                        }),
                        new WaitCommand(500)//,
                        // new FollowPath(zone3_3x, zone3_3y, zone3_3t, PositionCommand)

                );
            }
            DeFacut.addCommands(new FollowPath(common_1x, common_1y, common_1t, PositionCommand, 10, 10, 10),
                    new FollowPath(common_2x, common_2y, common_2t, PositionCommand),
                    new InstantCommand(()->{Robot.m_ServoIntake.setPosition(0.78);}),
                    new FollowPath(common_3x, common_3y, common_3t, PositionCommand),
                    new FollowPath(common_4x, common_4y, common_4t, PositionCommand),
                    new WaitCommand(1000),
                    new StackCommand(Robot, PositionCommand, common_4x, common_4y + 10, common_4t),
                    new FollowPath(common_5x, common_5y, common_5t, PositionCommand),
                    new FollowPath(common_6x, common_6y, common_6t, PositionCommand).alongWith(
                            new InstantCommand(()->{Robot.m_ServoIntake.setPosition(RobotHardware.IntakePos);})
                    )



            );
            CommandScheduler.getInstance().schedule(DeFacut);
            ok = true;
        }
        telemetry.addData("ZONAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA", Zone);

        telemetry.update();
        Robot.Update();
        super.run();

    }


}

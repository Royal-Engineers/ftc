    package org.firstinspires.ftc.teamcode.team20936.auto.autoCommands.BasicCommands;

    import com.arcrobotics.ftclib.command.CommandBase;

    import org.firstinspires.ftc.teamcode.team20936.auto.subsystems.Singleton_AutoSubsystem;

    public class GotoLinearHeading extends CommandBase{
        private final Singleton_AutoSubsystem autoSubsystem;

        private final double TargetX, TargetY, Degrees_TargetHeading;
        public GotoLinearHeading(double TargetX, double TargetY, double Degrees_TargetHeading)
        {
            autoSubsystem = Singleton_AutoSubsystem.getInstance();
            addRequirements(autoSubsystem);

            this.Degrees_TargetHeading = Degrees_TargetHeading;
            this.TargetX = TargetX;
            this.TargetY = TargetY;
        }

        @Override
        public void initialize()
        {
            autoSubsystem.gotoLinearHeading(TargetX, TargetY, Degrees_TargetHeading);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

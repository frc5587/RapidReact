package frc.robot.commands;

import org.frc5587.lib.auto.RamseteCommandWrapper;
import org.frc5587.lib.auto.AutoPath;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePistons;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LinebreakSensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoPaths {
    // subsystems
    private final Intake intake;
    private final IntakePistons intakePistons;
    private final Conveyor conveyor;
    private final Kicker rightKicker;
    private final Kicker leftKicker;
    private final LinebreakSensor linebreakSensor;
    private final Drivetrain drivetrain;
    private final Limelight limelight;
    private final Turret turret;
    private final Shooter shooter;

    // Auto Paths
    private final RamseteCommandWrapper first1;
    private final RamseteCommandWrapper first2;
    private final RamseteCommandWrapper first3;
    private final RamseteCommandWrapper first4;
    private final RamseteCommandWrapper first1_2;
    private final RamseteCommandWrapper first2_2;
    private final RamseteCommandWrapper first3_2;
    private final RamseteCommandWrapper first4_2;
    private final RamseteCommandWrapper second3;
    private final RamseteCommandWrapper second4;
    private final RamseteCommandWrapper third3;
    private final RamseteCommandWrapper third4;
    private final RamseteCommandWrapper firstshoot4;
    private final RamseteCommandWrapper firstshoot4_2;
    private final RamseteCommandWrapper finalshoot3;
    private final RamseteCommandWrapper finalshoot4;
    private final RamseteCommandWrapper firstSteal1;
    private final RamseteCommandWrapper firstSteal2;
    private final RamseteCommandWrapper secondSteal;
    private final RamseteCommandWrapper secondSteal_2;
    private final RamseteCommandWrapper stash;
    private final RamseteCommandWrapper stash_2;
    private final RamseteCommandWrapper taxi1;
    private final RamseteCommandWrapper taxi2;
    private final RamseteCommandWrapper taxi3;
    private final RamseteCommandWrapper taxi4;

    // define auto command groups here so they can be referenced anywhere
    public final Command pos1stash;
    public final Command pos2;
    public final Command pos3FourBall;
    public final Command pos4FourBall;
    public final Command pos1NoStash;
    public final Command pos2NoStash;
    public final Command pos3TwoBall;
    public final Command pos4TwoBall;

    SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    /**
     * Creates autonomous paths for 4 field positions using
     * {@link RamseteCommandWrapper}
     * and {@link AutoPath}, then links them to a {@link SendableChooser}.
     * 
     * Positions can be seen here:
     * <img src="./doc-files/autoposdiagram_small.png" width=100% />
     */
    public AutoPaths(Intake intake, IntakePistons intakePistons, Conveyor conveyor, Kicker rightKicker,
            Kicker leftKicker, LinebreakSensor linebreakSensor, Drivetrain drivetrain, Limelight limelight,
            Turret turret, Shooter shooter, ClimbThrottle climbCommand) {
        this.intake = intake;
        this.intakePistons= intakePistons;
        this.conveyor = conveyor;
        this.rightKicker = rightKicker;
        this.leftKicker= leftKicker;
        this.linebreakSensor = linebreakSensor;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.turret = turret;
        this.shooter = shooter;
        // Auto Paths
        first1 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("first 1"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        first2 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("first 2"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        first3 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("first 3"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        first4 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("first 4"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        first1_2 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("first 1"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        first2_2 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("first 2"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        first3_2 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("first 3"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        first4_2 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("first 4"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        second3 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("second 3"), AutoConstants.RAMSETE_CONSTANTS);
        second4 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("second 4"), AutoConstants.RAMSETE_CONSTANTS);
        third3 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("third 3"), AutoConstants.RAMSETE_CONSTANTS);
        third4 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("third 4"), AutoConstants.RAMSETE_CONSTANTS);
        firstshoot4 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("firstshoot 4"), AutoConstants.RAMSETE_CONSTANTS);
        firstshoot4_2 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("firstshoot 4"), AutoConstants.RAMSETE_CONSTANTS);
        finalshoot3 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("finalshoot 3"), AutoConstants.RAMSETE_CONSTANTS);
        finalshoot4 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("finalshoot 4"), AutoConstants.RAMSETE_CONSTANTS);
        firstSteal1 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("first steal 1"), AutoConstants.RAMSETE_CONSTANTS);
        firstSteal2 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("first steal 2"), AutoConstants.RAMSETE_CONSTANTS);
        secondSteal = new RamseteCommandWrapper(drivetrain,
                new AutoPath("second steal"), AutoConstants.RAMSETE_CONSTANTS);
        secondSteal_2 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("second steal"), AutoConstants.RAMSETE_CONSTANTS);
        stash = new RamseteCommandWrapper(drivetrain,
                new AutoPath("stash"), AutoConstants.RAMSETE_CONSTANTS);
        stash_2 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("stash"), AutoConstants.RAMSETE_CONSTANTS);
        taxi1 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("taxi 1"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        taxi2 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("taxi 2"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        taxi3 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("taxi 3"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        taxi4 = new RamseteCommandWrapper(drivetrain,
                new AutoPath("taxi 4"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();

        this.pos1stash = new SequentialCommandGroup(
                        intakeDuringPath(first1),
                        timedCommand(4, new SpinUpShooter(shooter, drivetrain, turret, limelight),new FireWhenReady(conveyor, rightKicker, leftKicker, shooter)),
                        intakeDuringPath(firstSteal1),
                        intakeDuringPath(firstSteal2),
                        stash,
                        timedCommand(2, new TopBallOut(conveyor, rightKicker, leftKicker, linebreakSensor, shooter)),
                        timedCommand(3, new BottomBallOut(intake, intakePistons, conveyor)));

        this.pos2 = new SequentialCommandGroup(
                    intakeDuringPath(first2),
                        timedCommand(4, new SpinUpShooter(shooter, drivetrain, turret, limelight),new FireWhenReady(conveyor, rightKicker, leftKicker, shooter)),
                        intakeDuringPath(secondSteal),
                        intakeDuringPath(secondSteal_2),
                        stash_2,
                        timedCommand(2, new TopBallOut(conveyor, rightKicker, leftKicker, linebreakSensor, shooter)),
                        timedCommand(3, new BottomBallOut(intake, intakePistons, conveyor)));

        this.pos3FourBall = new SequentialCommandGroup(
                        intakeDuringPath(first3),
                        timedCommand(4, new SpinUpShooter(shooter, drivetrain, turret, limelight),new FireWhenReady(conveyor, rightKicker, leftKicker, shooter)),
                        intakeDuringPath(second3),
                        intakeDuringPath(third3),
                        finalshoot3,
                        timedCommand(4, new SpinUpShooter(shooter, drivetrain, turret, limelight),new FireWhenReady(conveyor, rightKicker, leftKicker, shooter)));

        this.pos4FourBall = new SequentialCommandGroup(
                    intakeDuringPath(first4),
                        firstshoot4,
                        timedCommand(4, new SpinUpShooter(shooter, drivetrain, turret, limelight),new FireWhenReady(conveyor, rightKicker, leftKicker, shooter)),
                        intakeDuringPath(second4),
                        intakeDuringPath(third4),
                        finalshoot4,
                        timedCommand(4, new SpinUpShooter(shooter, drivetrain, turret, limelight),new FireWhenReady(conveyor, rightKicker, leftKicker, shooter)));

        this.pos1NoStash = new SequentialCommandGroup(
                    intakeDuringPath(first1_2),
                    timedCommand(4, new SpinUpShooter(shooter, drivetrain, turret, limelight),new FireWhenReady(conveyor, rightKicker, leftKicker, shooter)));

        this.pos2NoStash = new SequentialCommandGroup(
                    intakeDuringPath(first2_2),
                                timedCommand(4, new SpinUpShooter(shooter, drivetrain, turret, limelight),new FireWhenReady(conveyor, rightKicker, leftKicker, shooter)));

        this.pos3TwoBall = new SequentialCommandGroup(
                    intakeDuringPath(first3_2),
                    timedCommand(2, new SpinUpShooter(shooter, drivetrain, turret, limelight),new FireWhenReady(conveyor, rightKicker, leftKicker, shooter)));

        this.pos4TwoBall = new SequentialCommandGroup(
                intakeDuringPath(first4_2),
                firstshoot4_2,
                timedCommand(4, new SpinUpShooter(shooter, drivetrain, turret, limelight),new FireWhenReady(conveyor, rightKicker, leftKicker, shooter)));
        
        autoChooser.addOption("1st Position With Stash", pos1stash);
        autoChooser.addOption("2nd Position With Stash", pos2);
        autoChooser.addOption("3rd Position 4 Ball", pos3FourBall);
        autoChooser.addOption("4th Position 4 Ball", pos4FourBall);
        autoChooser.addOption("1st Position No Stash", pos1NoStash);
        autoChooser.addOption("2nd Position No Stash", pos2NoStash);
        autoChooser.addOption("3rd Position 2 Ball", pos3TwoBall);
        autoChooser.addOption("4th Position 2 Ball", pos4TwoBall);
        autoChooser.addOption("Taxi 1st Pos", taxi1);
        autoChooser.addOption("Taxi 2nd Pos", taxi2);
        autoChooser.addOption("Taxi 3rd Pos", taxi3);
        autoChooser.addOption("Taxi 4th Pos", taxi4);
        autoChooser.setDefaultOption("NO COMMAND", null);
        SmartDashboard.putData(autoChooser);
    }

    private Command timedCommand(double time, Command... commands){
        Command[] commandsWithWait = new Command[commands.length + 1];
        commandsWithWait[0] = new WaitCommand(time);

        for (int i = 0; i < commands.length; i++) {
            commandsWithWait[i + 1] = commands[i];
        }
        return new ParallelRaceGroup(commandsWithWait);
    }

    private Command intakeDuringPath(RamseteCommandWrapper path) {
        return new ParallelRaceGroup(path, new Index(intake, intakePistons, conveyor, rightKicker, leftKicker, linebreakSensor, drivetrain));
    }


    public SendableChooser<Command> getChooser() {
        return this.autoChooser;
    }

    public Command getSelectedCommand() {
        return autoChooser.getSelected();
    }
}

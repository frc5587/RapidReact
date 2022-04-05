package frc.robot.commands;

import org.frc5587.lib.auto.RamseteCommandWrapper;
import org.frc5587.lib.auto.AutoPath;
import org.frc5587.lib.auto.ConstrainedTrajectory;

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
    private RamseteCommandWrapper first1;
    private RamseteCommandWrapper first2;
    private RamseteCommandWrapper first3;
    private RamseteCommandWrapper first4;
    private RamseteCommandWrapper first1_2Ball;
    private RamseteCommandWrapper first2_2Ball;
    private RamseteCommandWrapper first3_2Ball;
    private RamseteCommandWrapper first4_2Ball;
    private RamseteCommandWrapper first3_3Ball;
    private RamseteCommandWrapper first4_3Ball;
    private RamseteCommandWrapper second3;
    private RamseteCommandWrapper second4;
    private RamseteCommandWrapper second3_3Ball;
    private RamseteCommandWrapper second4_3Ball;
    private RamseteCommandWrapper finalshoot;
    private RamseteCommandWrapper finalshoot_2;
    private RamseteCommandWrapper firstSteal1;
    private RamseteCommandWrapper firstSteal2;
    private RamseteCommandWrapper secondSteal;
    private RamseteCommandWrapper secondSteal_2;
    private RamseteCommandWrapper taxi1;
    private RamseteCommandWrapper taxi2;
    private RamseteCommandWrapper taxi3;
    private RamseteCommandWrapper taxi4;
    private RamseteCommandWrapper pos4_5BallPath1;
    private RamseteCommandWrapper pos4_5BallPath2;

    /* define auto command groups here so they can be referenced anywhere */
    /**
     * Starting with 1 pre-loaded cargo in position 1, grab another then shoot.
     * Stash two cargo from the opposing alliance in the corner of the field
     */
    public final Command pos1stash;
    /**
     * Starting with 1 pre-loaded cargo in position 2, grab another then shoot.
     * Stash two cargo from the opposing alliance in the corner of the field
     */
    public final Command pos2stash;
    /**
     * Get the closest ball and then shoot it. Next, get the one against the wall.
     * Back up and go to the terminal to get the ball in front of it, then shoot
     * both.
     */
    public final Command pos3FourBall;
    /**
     * Get the ball against the wall and back up to shoot it. Drive through the next
     * ball
     * on the way to the terminal, where the last ball is retrieved. Back up to
     * shoot both.
     */
    public final Command pos4FourBall;
    /**
     * Get the closest ball and then shoot it. Next, get the one against the wall
     * and shoot it.
     */
    public final Command pos3ThreeBall;
    /**
     * Get the ball against the wall and turn around to shoot it. Drive through the
     * next ball
     * and spin around to shoot.
     */
    public final Command pos4ThreeBall;
    /**
     * Get the closest ball and shoot that along with the pre-loaded cargo.
     */
    public final Command pos1NoStash;
    /**
     * Get the closest ball and shoot that along with the pre-loaded cargo.
     */
    public final Command pos2NoStash;
    /**
     * Get the closest ball and shoot that along with the pre-loaded cargo.
     */
    public final Command pos3TwoBall;
    /**
     * Get the closest ball and shoot that along with the pre-loaded cargo.
     */
    public final Command pos4TwoBall;
    /**
     * Gets the closets ball, then starts starting spinning up and firing as it head
     * towards the next near ball, pauses there to fire all of them, then continues
     * to the player station to get the two balls, the comes back and shoots
     */
    // public final Command pos4FiveBall;

    public final boolean usingPathPlannerPaths;
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
            Turret turret, Shooter shooter, ClimbThrottle climbCommand, boolean pathPlannerPaths) {
        this.intake = intake;
        this.intakePistons = intakePistons;
        this.conveyor = conveyor;
        this.rightKicker = rightKicker;
        this.leftKicker = leftKicker;
        this.linebreakSensor = linebreakSensor;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.turret = turret;
        this.shooter = shooter;
        this.usingPathPlannerPaths = pathPlannerPaths;

        // Cool Paths
        pos4_5BallPath1 = new RamseteCommandWrapper(drivetrain, new AutoPath("pos4 5ball path1", true), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        pos4_5BallPath2 = new RamseteCommandWrapper(drivetrain, new AutoPath("pos4 5ball path2", true), AutoConstants.RAMSETE_CONSTANTS);



        // Auto Paths
        if (pathPlannerPaths) {
            first2 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first 2", true), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first3 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first 3", true), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first4 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first 4", true), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first1_2Ball = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first 1", true), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first2_2Ball = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first 2", true), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first3_2Ball = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first 3", true), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first4_2Ball = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first 4", true), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first3_3Ball = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first 3", true), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first4_3Ball = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first 4", true), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            secondSteal = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("second 1", true), AutoConstants.RAMSETE_CONSTANTS);
            secondSteal_2 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("second 2", true), AutoConstants.RAMSETE_CONSTANTS);
            second3 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("second 3", true), AutoConstants.RAMSETE_CONSTANTS);
            second4 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("second 4", true), AutoConstants.RAMSETE_CONSTANTS);
            second3_3Ball = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("second 3 2", true), AutoConstants.RAMSETE_CONSTANTS);
            second4_3Ball = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("second 4 2", true), AutoConstants.RAMSETE_CONSTANTS);
            taxi1 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("taxi 1", true), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            taxi2 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("taxi 2", true), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            taxi3 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("taxi 3", true), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            taxi4 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("taxi 4", true), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        } else {
            first1 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first 1"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first2 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first 2"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first3 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first 3"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first4 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first 4"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first1_2Ball = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first 1"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first2_2Ball = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first 2"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first3_2Ball = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first 3"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first4_2Ball = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first 4"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first3_3Ball = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first 3"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first4_3Ball = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first 4"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            second3 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("second 3"), AutoConstants.RAMSETE_CONSTANTS);
            second4 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("second 4"), AutoConstants.RAMSETE_CONSTANTS);
            second3_3Ball = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("second 3 2"), AutoConstants.RAMSETE_CONSTANTS);
            second4_3Ball = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("second 4 2"), AutoConstants.RAMSETE_CONSTANTS);
            finalshoot = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("finalshoot 3 and 4"), AutoConstants.RAMSETE_CONSTANTS);
            finalshoot_2 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("finalshoot 3 and 4"), AutoConstants.RAMSETE_CONSTANTS);
            firstSteal1 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first steal 1"), AutoConstants.RAMSETE_CONSTANTS);
            firstSteal2 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("first steal 2"), AutoConstants.RAMSETE_CONSTANTS);
            secondSteal = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("second steal"), AutoConstants.RAMSETE_CONSTANTS);
            secondSteal_2 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("second steal"), AutoConstants.RAMSETE_CONSTANTS);
            taxi1 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("taxi 1"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            taxi2 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("taxi 2"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            taxi3 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("taxi 3"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            taxi4 = new RamseteCommandWrapper(drivetrain,
                    new AutoPath("taxi 4"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        }
        this.pos1stash = new SequentialCommandGroup(
                intakeDuringPath(first1),
                fullShootCommand(2),
                intakeDuringPath(firstSteal1),
                intakeDuringPath(secondSteal),
                timedCommand(2, new TopBallOut(conveyor, rightKicker, leftKicker, linebreakSensor, shooter)),
                timedCommand(3, new BottomBallOut(intake, intakePistons, conveyor)));

        this.pos2stash = new SequentialCommandGroup(
                intakeDuringPath(first2),
                fullShootCommand(2),
                intakeDuringPath(firstSteal2),
                intakeDuringPath(secondSteal_2),
                timedCommand(2, new TopBallOut(conveyor, rightKicker, leftKicker, linebreakSensor, shooter)),
                timedCommand(3, new BottomBallOut(intake, intakePistons, conveyor)));

        this.pos3FourBall = new SequentialCommandGroup(
                intakeDuringPath(first3),
                fullShootCommand(2),
                intakeDuringPath(second3),
                finalshoot,
                fullShootCommand(2));

        this.pos4FourBall = new SequentialCommandGroup(
                intakeDuringPath(first4),
                fullShootCommand(2),
                intakeDuringPath(second4),
                finalshoot_2,
                fullShootCommand(2));

        this.pos1NoStash = new SequentialCommandGroup(
                intakeDuringPath(first1_2Ball),
                fullShootCommand(2));

        this.pos2NoStash = new SequentialCommandGroup(
                intakeDuringPath(first2_2Ball),
                fullShootCommand(2));

        this.pos3TwoBall = new SequentialCommandGroup(
                intakeDuringPath(first3_2Ball),
                fullShootCommand(2));

        this.pos4TwoBall = new SequentialCommandGroup(
                intakeDuringPath(first4_2Ball),
                fullShootCommand(2));

        this.pos3ThreeBall = new SequentialCommandGroup(
                intakeDuringPath(first3_3Ball),
                fullShootCommand(2),
                intakeDuringPath(second3_3Ball),
                fullShootCommand(2));

        this.pos4ThreeBall = new SequentialCommandGroup(
                intakeDuringPath(first4_3Ball),
                fullShootCommand(2),
                intakeDuringPath(second4_3Ball),
                fullShootCommand(2));

        // this.pos4FiveBall = new SequentialCommandGroup(
        //     new ParallelRaceGroup(
        //         new SequentialCommandGroup(pos4_5BallPath1, new WaitCommand(1)),
        //         new Index(intake, intakePistons, conveyor, linebreakSensor, drivetrain)),
        //         new SpinUpShooter(shooter, drivetrain, limelight),
        //         new SequentialCommandGroup(
        //             new WaitCommand(3.5),
        //             new FireWhenReady(conveyor, rightKicker, leftKicker, shooter, limelight)
        //     ),
        //     new ParallelCommandGroup(
        //         new ParallelRaceGroup(pos4_5BallPath2, new Index(intake, intakePistons, conveyor, linebreakSensor, drivetrain)),
        //         new SequentialCommandGroup(new WaitCommand(6), new SpinUpShooter(shooter, drivetrain, limelight)),
        //         new FireWhenReady(conveyor, rightKicker, leftKicker, shooter, limelight)
        //     )
        // );
        
        autoChooser.addOption("1st Position With Stash", pos1stash);
        autoChooser.addOption("2nd Position With Stash", pos2stash);
        autoChooser.addOption("3rd Position 4 Ball", pos3FourBall);
        autoChooser.addOption("4th Position 4 Ball", pos4FourBall);
        autoChooser.addOption("1st Position No Stash", pos1NoStash);
        autoChooser.addOption("2nd Position No Stash", pos2NoStash);
        autoChooser.addOption("3rd Position 2 Ball", pos3TwoBall);
        autoChooser.addOption("4th Position 2 Ball", pos4TwoBall);
        autoChooser.addOption("3rd Position 3 Ball", pos3ThreeBall);
        autoChooser.addOption("4th Position 3 Ball", pos4ThreeBall);
        // autoChooser.addOption("4th Position 5 Ball", pos4FiveBall);
        autoChooser.addOption("Taxi 1st Pos", taxi1);
        autoChooser.addOption("Taxi 2nd Pos", taxi2);
        autoChooser.addOption("Taxi 3rd Pos", taxi3);
        autoChooser.addOption("Taxi 4th Pos", taxi4);
        autoChooser.setDefaultOption("NO COMMAND", null);
        SmartDashboard.putData(autoChooser);
    }

    private Command timedCommand(double time, Command... commands) {
        Command[] commandsWithWait = new Command[commands.length + 1];
        commandsWithWait[0] = new WaitCommand(time);

        for (int i = 0; i < commands.length; i++) {
            commandsWithWait[i + 1] = commands[i];
        }
        return new ParallelRaceGroup(commandsWithWait);
    }

    private Command fullShootCommand(double time) {
        return new SequentialCommandGroup(
                timedCommand(time, new SpinUpShooter(shooter, drivetrain, limelight),
                        new FireWhenReady(conveyor, rightKicker, leftKicker, shooter, limelight)));
    }

    private Command intakeDuringPath(RamseteCommandWrapper path) {
        return new ParallelRaceGroup(
                path,
                new Index(intake, intakePistons, conveyor, linebreakSensor, drivetrain));
    }

    public SendableChooser<Command> getChooser() {
        return this.autoChooser;
    }

    public Command getSelectedCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Creates new RamseteCommandWrappers for all paths using a
     * {@link ConstrainedTrajectory}
     */
    public void useConstrainedTrajectories() {
        if (usingPathPlannerPaths) {
            first1 = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 1", true).trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first2 = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 2", true).trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first3 = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 3", true).trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first4 = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 4", true).trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first1_2Ball = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 1", true).trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first2_2Ball = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 2", true).trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first3_2Ball = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 3", true).trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first4_2Ball = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 4", true).trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first3_3Ball = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 3", true).trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first4_3Ball = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 4", true).trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            secondSteal = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("second 1", true).trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS);
            secondSteal_2 = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("second 2", true).trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS);
            second3 = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("second 3", true).trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS);
            second4 = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("second 4", true).trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS);
            second3_3Ball = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("second 3 2", true).trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS);
            second4_3Ball = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("second 4 2", true).trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS);
        } else {
            first1 = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 1").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first2 = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 2").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first3 = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 3").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first4 = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 4").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first1_2Ball = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 1").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first2_2Ball = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 2").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first3_2Ball = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 3").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first4_2Ball = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 4").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first3_3Ball = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 3").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            first4_3Ball = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first 4").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
            second3 = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("second 3").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS);
            second4 = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("second 4").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS);
            second3_3Ball = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("second 3 2").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS);
            second4_3Ball = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("second 4 2").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS);
            finalshoot = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("finalshoot 3 and 4").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS);
            finalshoot_2 = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("finalshoot 3 and 4").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS);
            firstSteal1 = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first steal 1").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS);
            firstSteal2 = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("first steal 2").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS);
            secondSteal = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("secondSteal").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS);
            secondSteal_2 = new RamseteCommandWrapper(drivetrain, ConstrainedTrajectory.constrain(
                    new AutoPath("secondSteal").trajectory, AutoConstants.TRAJECTORY_CONSTRAINTS),
                    AutoConstants.RAMSETE_CONSTANTS);
        }
    }
}
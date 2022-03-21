package frc.robot.commands;

import org.frc5587.lib.auto.*;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;

public class AutoPaths {
    // Auto Paths
    private final RamseteCommandWrapper first1;
    private final RamseteCommandWrapper first2;
    private final RamseteCommandWrapper first3;
    private final RamseteCommandWrapper first4;
    private final RamseteCommandWrapper first12;
    private final RamseteCommandWrapper first22;
    private final RamseteCommandWrapper first32;
    private final RamseteCommandWrapper first42;
    private final RamseteCommandWrapper second3;
    private final RamseteCommandWrapper second4;
    private final RamseteCommandWrapper third3;
    private final RamseteCommandWrapper third4;
    private final RamseteCommandWrapper firstshoot4;
    private final RamseteCommandWrapper finalshoot3;
    private final RamseteCommandWrapper finalshoot4;
    private final RamseteCommandWrapper firstSteal1;
    private final RamseteCommandWrapper firstSteal2;
    private final RamseteCommandWrapper secondSteal;
    private final RamseteCommandWrapper secondSteal2;
    private final RamseteCommandWrapper stash;
    private final RamseteCommandWrapper stash2;
    private final RamseteCommandWrapper taxi1;
    private final RamseteCommandWrapper taxi2;
    private final RamseteCommandWrapper taxi3;
    private final RamseteCommandWrapper taxi4;

    // define auto command groups here so they can be referenced anywhere
    public final Command pos1;
    public final Command pos2;
    public final Command pos3FourBall;
    public final Command pos4FourBall;
    public final Command pos1NoStash;
    public final Command pos2NoStash;
    public final Command pos3TwoBall;
    public final Command pos4TwoBall;

    SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    /**
     * Creates autonomous paths for 4 field positions using {@link RamseteCommandWrapper}
     * and {@link AutoPath}, then links them to a {@link SendableChooser}.
     * 
     * Positions can be seen here:
     * <img src="./doc-files/autoposdiagram_small.png" width=100% />
     */
    public AutoPaths(Intake intake, IntakePistons intakePistons, Conveyor conveyor, Kicker rightKicker,
    Kicker leftKicker, LinebreakSensor linebreakSensor, Drivetrain drivetrain, Limelight limelight, 
    Turret turret, Shooter shooter, ClimbThrottle climbCommand) {
        // Auto Paths
        first1 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("first 1"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        first2 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("first 2"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        first3 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("first 3"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        first4 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("first 4"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        first12 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("first 1"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        first22 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("first 2"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        first32 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("first 3"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        first42 = new RamseteCommandWrapper(drivetrain,
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
        secondSteal2 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("second steal"), AutoConstants.RAMSETE_CONSTANTS);
        stash = new RamseteCommandWrapper(drivetrain,
            new AutoPath("stash"), AutoConstants.RAMSETE_CONSTANTS);
        stash2 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("stash"), AutoConstants.RAMSETE_CONSTANTS);
        taxi1 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("taxi 1"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        taxi2 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("taxi 2"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        taxi3 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("taxi 3"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        taxi4 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("taxi 4"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();

        this.pos1 = new ParallelCommandGroup(
            new LockTurret(turret, limelight, climbCommand),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    first1
                ),
                new ParallelRaceGroup(
                    new SpinUpShooter(shooter, drivetrain, turret, limelight),
                    new FireWhenReady(conveyor, rightKicker, leftKicker, shooter),
                    new WaitCommand(4)
                ),
                new ParallelRaceGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    firstSteal1
                ),
                new ParallelRaceGroup (
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    secondSteal
                ),
                stash,
                new ParallelRaceGroup(
                    new TopBallOut(conveyor, rightKicker, leftKicker, linebreakSensor, shooter),
                    new WaitCommand(2)
                ),
                new ParallelRaceGroup(
                    new BottomBallOut(intake, intakePistons, conveyor),
                    new WaitCommand(3)
                )
            )
        );

        this.pos2 = new ParallelCommandGroup(
            new LockTurret(turret, limelight, climbCommand),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    first2
                ),
                new ParallelRaceGroup(
                    new SpinUpShooter(shooter, drivetrain, turret, limelight),
                    new FireWhenReady(conveyor, rightKicker, leftKicker, shooter),
                    new WaitCommand(4)
                ),
                new ParallelRaceGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    firstSteal2
                ),
                new ParallelRaceGroup (
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    secondSteal2
                ),
                stash2,
                new ParallelRaceGroup(
                    new TopBallOut(conveyor, rightKicker, leftKicker, linebreakSensor, shooter),
                    new WaitCommand(2)
                ),
                new ParallelRaceGroup(
                    new BottomBallOut(intake, intakePistons, conveyor),
                    new WaitCommand(3)
                )
            )
        );

        this.pos3FourBall = new ParallelCommandGroup(
            new LockTurret(turret, limelight, climbCommand),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    first3
                ),
                new ParallelCommandGroup(
                    new SpinUpShooter(shooter, drivetrain, turret, limelight),
                    new FireWhenReady(conveyor, rightKicker, leftKicker, shooter),
                    new WaitCommand(4)
                ), 
                new ParallelRaceGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    second3
                ),
                new ParallelRaceGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    third3
                ),
                finalshoot3,
                new ParallelRaceGroup(
                    new SpinUpShooter(shooter, drivetrain, turret, limelight),
                    new FireWhenReady(conveyor, rightKicker, leftKicker, shooter),
                    new WaitCommand(4)
                )
            )        
        );

        this.pos4FourBall = new ParallelCommandGroup(
            new LockTurret(turret, limelight, climbCommand),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    first4
                ),
                firstshoot4,
                new ParallelRaceGroup(
                    new SpinUpShooter(shooter, drivetrain, turret, limelight),
                    new FireWhenReady(conveyor, rightKicker, leftKicker, shooter),
                    new WaitCommand(4)
                ),
                new ParallelRaceGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    second4
                ),
                new ParallelRaceGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    third4
                ),
                finalshoot4,
                new ParallelRaceGroup(
                    new SpinUpShooter(shooter, drivetrain, turret, limelight),
                    new FireWhenReady(conveyor, rightKicker, leftKicker, shooter),
                    new WaitCommand(4)
                )
            )
        );

        this.pos1NoStash = new ParallelCommandGroup(
            new LockTurret(turret, limelight, climbCommand),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    first12
                ),
                new ParallelRaceGroup(
                    new SpinUpShooter(shooter, drivetrain, turret, limelight),
                    new FireWhenReady(conveyor, rightKicker, leftKicker, shooter),
                    new WaitCommand(4)
                )
            )
        );

        this.pos2NoStash = new ParallelCommandGroup(
            new LockTurret(turret, limelight, climbCommand),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    first22
                ),
                new ParallelRaceGroup(
                    new SpinUpShooter(shooter, drivetrain, turret, limelight),
                    new FireWhenReady(conveyor, rightKicker, leftKicker, shooter),
                    new WaitCommand(4)
                )
            )
        );

        this.pos3TwoBall = new ParallelCommandGroup(
            new LockTurret(turret, limelight, climbCommand),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    first32
                ),
                new ParallelCommandGroup(
                    new SpinUpShooter(shooter, drivetrain, turret, limelight),
                    new FireWhenReady(conveyor, rightKicker, leftKicker, shooter),
                    new WaitCommand(4)
                )
            )        
        );

        this.pos4TwoBall = new ParallelCommandGroup(
            new LockTurret(turret, limelight, climbCommand),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    first42
                ),
                firstshoot4,
                new ParallelRaceGroup(
                    new SpinUpShooter(shooter, drivetrain, turret, limelight),
                    new FireWhenReady(conveyor, rightKicker, leftKicker, shooter),
                    new WaitCommand(4)
                )
            )
        );

        Command emptyCommand = new SequentialCommandGroup();
        
        autoChooser.addOption("1st Position With Stash", pos1);
        autoChooser.addOption("2nd Position With Stash", pos2);
        autoChooser.addOption("3rd Position 4 Ball", pos3FourBall);
        autoChooser.addOption("4th Position 4 Ball", pos4FourBall);
        autoChooser.addOption("1st Position No Stash", pos1);
        autoChooser.addOption("2nd Position No Stash", pos2);
        autoChooser.addOption("3rd Position 2 Ball", pos3FourBall);
        autoChooser.addOption("4th Position 2 Ball", pos4FourBall);
        autoChooser.addOption("Taxi 1st Pos", taxi1);
        autoChooser.addOption("Taxi 2nd Pos", taxi2);
        autoChooser.addOption("Taxi 3rd Pos", taxi3);
        autoChooser.addOption("Taxi 4th Pos", taxi4);
        autoChooser.setDefaultOption("NO COMMAND", emptyCommand);
        SmartDashboard.putData(autoChooser);
    }

    public SendableChooser<Command> getChooser() {
        return this.autoChooser;
    }

    public Command getSelectedCommand() {
        return autoChooser.getSelected();
    }
}

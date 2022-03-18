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
    // define auto command groups here so they can be referenced anywhere
    public final Command pos1;
    public final Command pos2;
    public final Command pos3;
    public final Command pos4;

    SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    /**
     * Creates autonomous paths for 4 field positions using {@link RamseteCommandWrapper}
     * and {@link AutoPath}, then links them to a {@link SendableChooser}.
     * 
     * Positions can be seen here:
     * <img src="./doc-files/autoposdiagram.png" width=100% />
     */
    public AutoPaths(Intake intake, IntakePistons intakePistons, Conveyor conveyor, Kicker rightKicker,     Kicker leftKicker, LinebreakSensor linebreakSensor, Drivetrain drivetrain, Limelight limelight, Turret turret, Shooter shooter) {
        // Auto Paths
        first1 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("first 1"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        first2 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("first 2"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        first3 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("first 3"), AutoConstants.RAMSETE_CONSTANTS).setOdometryToFirstPoseOnStart();
        first4 = new RamseteCommandWrapper(drivetrain,
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

        this.pos1 = new ParallelCommandGroup(
            new LockTurret(turret, limelight, drivetrain),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    first1),
                new SpinUpShooter(shooter, limelight),
                new FireWhenReady(conveyor, rightKicker, leftKicker, shooter),
                new ParallelCommandGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    firstSteal1),
                new ParallelCommandGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    secondSteal),
                stash,
                new TopBallOut(conveyor, rightKicker, leftKicker, linebreakSensor, shooter),
                new BottomBallOut(intake, intakePistons, conveyor)));

        this.pos2 = new ParallelCommandGroup(
            new LockTurret(turret, limelight, drivetrain),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    first2),
                new SpinUpShooter(shooter, limelight),
                new FireWhenReady(conveyor, rightKicker, leftKicker, shooter),
                new ParallelCommandGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    firstSteal2),
                new ParallelCommandGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    secondSteal2),
                stash2,
                new TopBallOut(conveyor, rightKicker, leftKicker, linebreakSensor, shooter),
                new BottomBallOut(intake, intakePistons, conveyor)));

        this.pos3 = new ParallelCommandGroup(
            new LockTurret(turret, limelight, drivetrain),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    first3),
                new SpinUpShooter(shooter, limelight),
                new FireWhenReady(conveyor, rightKicker, leftKicker, shooter),
                new ParallelCommandGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    second3),
                new ParallelCommandGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    third3),
                finalshoot3,
                new SpinUpShooter(shooter, limelight),
                new FireWhenReady(conveyor, rightKicker, leftKicker, shooter)));

        this.pos4 = new ParallelCommandGroup(
            new LockTurret(turret, limelight, drivetrain),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    first4),
                firstshoot4,
                new SpinUpShooter(shooter, limelight),
                new FireWhenReady(conveyor, rightKicker, leftKicker, shooter),
                new ParallelCommandGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    second4),
                new ParallelCommandGroup(
                    new Index(intake, intakePistons, conveyor, rightKicker,
                        leftKicker, linebreakSensor, drivetrain),
                    third4),
                finalshoot4,
                new SpinUpShooter(shooter, limelight),
                new FireWhenReady(conveyor, rightKicker, leftKicker, shooter)));
        
        autoChooser.addOption("1st Position", pos1);
        autoChooser.addOption("2nd Position", pos2);
        autoChooser.addOption("3rd Position", pos3);
        autoChooser.addOption("4th Position", pos4);
        autoChooser.setDefaultOption("1st Position", pos1);
        SmartDashboard.putData(autoChooser);
    }

    public SendableChooser<Command> getChooser() {
        return this.autoChooser;
    }

    public Command getSelectedCommand() {
        return autoChooser.getSelected();
    }
}

package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.HubStraightBack;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public final class AutoRoutines {
    private final CommandSwerveDrivetrain drivetrain;
    private final IntakeSubsystem intake;
    private final FloorSubsystem floor;
    private final FeederSubsystem feeder;
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final HangerSubsystem hanger;
    private final LimelightSubsystem limelight;

    private final SubsystemCommands subsystemCommands;

    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    public AutoRoutines(
        CommandSwerveDrivetrain drivetrain,
        IntakeSubsystem intake,
        FloorSubsystem floor,
        FeederSubsystem feeder,
        ShooterSubsystem shooter,
        HoodSubsystem hood,
        HangerSubsystem hanger,
        LimelightSubsystem limelight
    ) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;
        this.limelight = limelight;

        this.subsystemCommands = new SubsystemCommands(drivetrain, intake, floor, feeder, shooter, hood, hanger);

        this.autoFactory = drivetrain.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    public void configure() {
        autoChooser.addRoutine("Hub Straight Back", this::hubStraightBackRoutine);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private AutoRoutine hubStraightBackRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("Hub Straight Back");
        final AutoTrajectory trajectory = HubStraightBack.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                trajectory.resetOdometry(),
                trajectory.cmd()
            )
        );

        return routine;
    }
}

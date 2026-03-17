package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.CenterPickup;
import static frc.robot.generated.ChoreoTraj.CornerHubBump;
import static frc.robot.generated.ChoreoTraj.HubLongStraightBack;
import static frc.robot.generated.ChoreoTraj.HubShortStraightBack;
import static frc.robot.generated.ChoreoTraj.HubtoDepot;
import static frc.robot.generated.ChoreoTraj.NoMove;

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
        autoChooser.addRoutine("No Shot", this::noShot);
        autoChooser.addRoutine("Hub Short Shot Back", this::hubShortShotAuton);
        autoChooser.addRoutine("Hub Long Shot Back", this::hubLongShotAuton);
        autoChooser.addRoutine("Corner Hub 2 Center", this::cornerHubCenterAuton);
        autoChooser.addRoutine("Hub To Depot", this::hubToDepotAuton);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    
    private AutoRoutine noShot() {
        final AutoRoutine routine = autoFactory.newRoutine("No Shot");
        final AutoTrajectory driveBack = NoMove.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                // Shoot all fuel into hub
                //subsystemCommands.hubShotAuton().withTimeout(5),
                // Drive straight back
                driveBack.resetOdometry(),
                // Drive back and raise climber arms at the same time
                Commands.parallel(
                    driveBack.cmd(),
                    hanger.positionCommand(HangerSubsystem.Position.EXTEND_HOPPER)
                )
            )
        );
        return routine;
    }

    
    private AutoRoutine hubShortShotAuton() {
        final AutoRoutine routine = autoFactory.newRoutine("Hub Short Shot Back");
        final AutoTrajectory driveBack = HubShortStraightBack.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                // Shoot all fuel into hub
                subsystemCommands.hubShotAuton().withTimeout(5),
                // Drive straight back
                driveBack.resetOdometry(),
                // Drive back and raise climber arms at the same time
                Commands.parallel(
                    driveBack.cmd(),
                    hanger.positionCommand(HangerSubsystem.Position.EXTEND_HOPPER)
                )
            )
        );

        return routine;
    }



    private AutoRoutine cornerHubCenterAuton() {
        final AutoRoutine routine = autoFactory.newRoutine("Corner Hub 2 Center");
        final AutoTrajectory driveBump = CornerHubBump.asAutoTraj(routine);
        final AutoTrajectory centerPickup = CenterPickup.asAutoTraj(routine);


        routine.active().onTrue(
            Commands.sequence(
                // Shoot all fuel into hub
                subsystemCommands.hubShotAuton().withTimeout(5),
                // Need it to follow the trajectory here.... how to do that?
                driveBump.resetOdometry(),
                // Drive back and raise climber arms at the same time
                Commands.parallel(
                    driveBump.cmd(),
                    hanger.positionCommand(HangerSubsystem.Position.EXTEND_HOPPER)
                ),
                // Drive to center and pick up fuel
                Commands.parallel(
                    centerPickup.cmd(),
                    intake.intakeCommand()
                )
            )
        );

        return routine;
    }


    private AutoRoutine hubLongShotAuton() {
        final AutoRoutine routine = autoFactory.newRoutine("Hub Long Shot Back");
        final AutoTrajectory driveBack = HubLongStraightBack.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                // Shoot all fuel into hub
                subsystemCommands.hubShotAuton().withTimeout(5),
                // Drive straight back
                driveBack.resetOdometry(),
                // Drive back and raise climber arms at the same time
                Commands.parallel(
                    driveBack.cmd(),
                    hanger.positionCommand(HangerSubsystem.Position.EXTEND_HOPPER)
                )
            )
        );

        return routine;
    }


    private AutoRoutine hubToDepotAuton() {
        final AutoRoutine routine = autoFactory.newRoutine("Hub To Depot");
        final AutoTrajectory driveToDepot = HubtoDepot.asAutoTraj(routine);
        //TODO: Create DepotToHub trajectory in Choreo, then uncomment below
        //final AutoTrajectory driveToHub = DepotToHub.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                // Shoot all fuel into hub
                subsystemCommands.hubShotAuton().withTimeout(5),
                // Reset odometry before driving
                driveToDepot.resetOdometry(),
                // Drive to depot while extending hopper
                Commands.parallel(
                    driveToDepot.cmd(),
                    hanger.positionCommand(HangerSubsystem.Position.EXTEND_HOPPER)
                ),
                // Gather fuel at depot
                intake.intakeCommand().withTimeout(3)
                //TODO: Uncomment after creating DepotToHub trajectory in Choreo
                // Drive back to hub center
                //driveToHub.cmd(),
                // Shoot gathered fuel
                //subsystemCommands.hubShotAuton().withTimeout(5)
            )
        );

        return routine;
    }
    
}

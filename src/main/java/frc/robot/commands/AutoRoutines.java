package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.Bump2Center;
import static frc.robot.generated.ChoreoTraj.Bump2CornerHubShott;
import static frc.robot.generated.ChoreoTraj.Bump2ShootEndwithTurn;
import static frc.robot.generated.ChoreoTraj.BumpBack2Center;
import static frc.robot.generated.ChoreoTraj.Center2Bump;
import static frc.robot.generated.ChoreoTraj.CenterPickup;
import static frc.robot.generated.ChoreoTraj.CornerHubBump;
import static frc.robot.generated.ChoreoTraj.Depot2Shoot;
import static frc.robot.generated.ChoreoTraj.Depot2ShootEndTurn;
import static frc.robot.generated.ChoreoTraj.HubLongStraightBack;
import static frc.robot.generated.ChoreoTraj.HubShortStraightBack;

import static frc.robot.generated.ChoreoTraj.NoMove;
import static frc.robot.generated.ChoreoTraj.RightBump2Center;
import static frc.robot.generated.ChoreoTraj.RightBump2CornerHubShott_copy1;
import static frc.robot.generated.ChoreoTraj.RightBumpBack2Center;
import static frc.robot.generated.ChoreoTraj.RightCenter2Bump;
import static frc.robot.generated.ChoreoTraj.RightCenterPickup;
import static frc.robot.generated.ChoreoTraj.TrenchToDepotFast;
import static frc.robot.generated.ChoreoTraj.TrenchToDepotSlow;

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
//import frc.robot.subsystems.HangerSubsystem;
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
    //private final HangerSubsystem hanger;
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
        //HangerSubsystem hanger,
        LimelightSubsystem limelight
    ) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        //this.hanger = hanger;
        this.limelight = limelight;

        this.subsystemCommands = new SubsystemCommands(drivetrain, intake, floor, feeder, shooter, hood);//hanger

        this.autoFactory = drivetrain.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    public void configure() {
        autoChooser.addRoutine("No Shot", this::noShot);
        autoChooser.addRoutine("Hub Short Shot Back", this::hubShortShotAuton);
        autoChooser.addRoutine("Hub Long Shot Back", this::hubLongShotAuton);
        autoChooser.addRoutine("Corner Hub 2 Center", this::cornerHubCenterAuton);
        autoChooser.addRoutine("Trench To Depot", this::trenchToDepotAuton);
        autoChooser.addRoutine("LeftBump To Center", this::bumpToCenterAuton);
        autoChooser.addRoutine("RightBump To Center", this::rightBumpToCenterAuton);

        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    
    private AutoRoutine noShot() {
        final AutoRoutine routine = autoFactory.newRoutine("No Shot");
        final AutoTrajectory driveBack = NoMove.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                // Shoot all fuel into hub
                //subsystemCommands.hubShotAuton().withTimeout(3.3),
                // Drive straight back
                driveBack.resetOdometry(),
                // Drive back
                driveBack.cmd()
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
                subsystemCommands.hubShotAuton().withTimeout(3.3),
                // Drive straight back
                driveBack.resetOdometry(),
                // Drive back
                Commands.sequence(
                    driveBack.cmd(),
                    intake.deployCommand() 
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
                subsystemCommands.hubShotAuton().withTimeout(3.3),
        
                driveBump.resetOdometry(),
                // Drive to bump
                driveBump.cmd(),
                // Drive to center and pick up fuel
                Commands.parallel(
                    centerPickup.cmd(),
                    intake.intakeCommand()
                )
            )
        );

        return routine;
    }



    private AutoRoutine bumpToCenterAuton() {
        final AutoRoutine routine = autoFactory.newRoutine("Bump 2 Center");
        final AutoTrajectory driveBump = BumpBack2Center.asAutoTraj(routine);
        final AutoTrajectory driveBumpToCenter = Bump2Center.asAutoTraj(routine);
        final AutoTrajectory centerPickup = CenterPickup.asAutoTraj(routine);
        final AutoTrajectory centerToBump = Center2Bump.asAutoTraj(routine);
        final AutoTrajectory bumpToShoot = Bump2CornerHubShott.asAutoTraj(routine);
        final AutoTrajectory endTurn = Bump2ShootEndwithTurn.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                // Shoot all fuel into hub
        
                driveBump.resetOdometry(),
                // Drive over bump
                driveBump.cmd(),

                
                Commands.parallel(
                    Commands.sequence(
                        driveBumpToCenter.resetOdometry(),
                        driveBumpToCenter.cmd()
                    ),
                    intake.intakeCommand().alongWith(floor.feedCommand()).withTimeout(2.0).withName("IntakeAndFeed")

                ),
                                
                // Drive to center and pick up fuel
                Commands.parallel(
                    centerPickup.cmd(),
                    //intake.intakeCommand().withTimeout(4.0)
                    intake.intakeCommand().alongWith(floor.feedCommand()).withTimeout(4.5).withName("IntakeAndFeed") //WAS 4.0
                ),

                centerToBump.cmd(),
                bumpToShoot.resetOdometry(),
                Commands.parallel(
                    bumpToShoot.cmd(),
                    subsystemCommands.briefReverse().withTimeout(.25)
                ),
                drivetrain.stopCommand(),

                //drive to corner of hub, vision align, then shoot
                subsystemCommands.visionAlignAndShootAuton().withTimeout(6.0), //WAS 5.0
                drivetrain.stopCommand(),
                subsystemCommands.hubShot().withTimeout(8.0),

                //subsystemCommands.hubShotCenter().withTimeout(10.0), // added for finals Livonia to avoid alliance partner
                //endTurn.cmd(), //for starting in Field oriented teleop
                drivetrain.stopCommand()

            )
                  
        );

        return routine;
    }


private AutoRoutine rightBumpToCenterAuton() {
        final AutoRoutine routine = autoFactory.newRoutine("RightBump 2 Center");
        final AutoTrajectory driveBump = RightBumpBack2Center.asAutoTraj(routine);
        final AutoTrajectory driveBump2Center = RightBump2Center.asAutoTraj(routine);
        final AutoTrajectory centerPickup = RightCenterPickup.asAutoTraj(routine);
        final AutoTrajectory centerToBump = RightCenter2Bump.asAutoTraj(routine);
        final AutoTrajectory bumpToShoot = RightBump2CornerHubShott_copy1.asAutoTraj(routine);
        //final AutoTrajectory endTurn = Bump2ShootEndwithTurn.asAutoTraj(routine);



        routine.active().onTrue(
            Commands.sequence(
                // Shoot all fuel into hub
        
                driveBump.resetOdometry(),
                // Drive over bump
                driveBump.cmd(),

                driveBump2Center.resetOdometry(),

                Commands.parallel(
                    driveBump2Center.cmd(),
                    intake.intakeCommand().alongWith(floor.feedCommand()).withTimeout(2.0).withName("IntakeAndFeed")

                ),
                
                // Drive to center and pick up fuel
                Commands.parallel(
                    centerPickup.cmd(),
                    //intake.intakeCommand().withTimeout(3.0)
                    intake.intakeCommand().alongWith(floor.feedCommand()).withTimeout(4.0).withName("IntakeAndFeed")

                ),

                centerToBump.cmd(),
                bumpToShoot.resetOdometry(),
                Commands.parallel(
                    bumpToShoot.cmd(),
                    subsystemCommands.briefReverse().withTimeout(.25)
                ),
                drivetrain.stopCommand(),

                 //drive to corner of hub, vision align, then shoot
                //subsystemCommands.hubShot().withTimeout(5.0),
                
                subsystemCommands.hubShotCenter().withTimeout(10.0),
               //subsystemCommands.visionAlignAndShootAuton().withTimeout(8.0),
                //endTurn.cmd(), //for starting in Field oriented teleop
                drivetrain.stopCommand()

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
                subsystemCommands.hubShotAuton().withTimeout(3.3),
                // Drive straight back
                driveBack.resetOdometry(),
                // Drive back
                Commands.sequence(
                    driveBack.cmd(),
                    intake.deployCommand()  //was intakeCommand
                )
            )
        );

        return routine;
    }


    private AutoRoutine trenchToDepotAuton() {
        final AutoRoutine routine = autoFactory.newRoutine("Trench To Depot");
        final AutoTrajectory driveToDepotFast = TrenchToDepotFast.asAutoTraj(routine);
        final AutoTrajectory driveToDepotSlow = TrenchToDepotSlow.asAutoTraj(routine);
        final AutoTrajectory driveToShoot = Depot2Shoot.asAutoTraj(routine);
        final AutoTrajectory endTurn = Depot2ShootEndTurn.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                // Shoot from trench
                subsystemCommands.trenchShotAuton().withTimeout(3.3),
                // Lower intake while stationary
                intake.deployCommand(),
                // Drive to depot
                driveToDepotFast.resetOdometry(),
                driveToDepotFast.cmd(),
                // Slow approach along wall with rollers running
                Commands.parallel(
                    driveToDepotSlow.cmd(),
                    //intake.intakeCommand().withTimeout(6.0)
                    intake.intakeCommand().alongWith(floor.feedCommand()).withTimeout(4.0).withName("IntakeAndFeed")
                ),
                // Reset odometry before depot-to-shoot to correct drift
                // driveToShoot.resetOdometry(),
                // Drive to sweet spot shooting position
                Commands.parallel(
                    driveToShoot.cmd(),
                    subsystemCommands.briefReverse().withTimeout(.25)
                ),

                drivetrain.stopCommand(),
                // Vision align then shoot gathered fuel
                subsystemCommands.sweetSpot().withTimeout(4.5),
                //subsystemCommands.visionAlignAndShootAuton().withTimeout(4.5),
                endTurn.cmd(), //for starting in Field oriented teleop
                drivetrain.stopCommand()
            )
        );

        return routine;
    }
    
}

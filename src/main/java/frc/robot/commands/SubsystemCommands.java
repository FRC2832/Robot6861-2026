package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public final class SubsystemCommands {
    private final CommandSwerveDrivetrain swerve;
    private final IntakeSubsystem intake;
    private final FloorSubsystem floor;
    private final FeederSubsystem feeder;
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final HangerSubsystem hanger;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    public SubsystemCommands(
        CommandSwerveDrivetrain swerve,
        IntakeSubsystem intake,
        FloorSubsystem floor,
        FeederSubsystem feeder,
        ShooterSubsystem shooter,
        HoodSubsystem hood,
        HangerSubsystem hanger,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;

        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
    }

    public SubsystemCommands(
        CommandSwerveDrivetrain swerve,
        IntakeSubsystem intake,
        FloorSubsystem floor,
        FeederSubsystem feeder,
        ShooterSubsystem shooter,
        HoodSubsystem hood,
        HangerSubsystem hanger
    ) {
        this(
            swerve,
            intake,
            floor,
            feeder,
            shooter,
            hood,
            hanger,
            () -> 0,
            () -> 0
        );
    }

    public Command aimAndShoot() {
        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);
        return Commands.parallel(
            aimAndDriveCommand,
            Commands.waitSeconds(0.25)
                .andThen(prepareShotCommand),
            Commands.waitUntil(() -> aimAndDriveCommand.isAimed() && prepareShotCommand.isReadyToShoot())
                .andThen(feed().withName("Aim Feed"))
        ).withName("Aim And Shoot");
    }

    public Command shootManually() {
        return shooter.dashboardSpinUpCommand()
            .andThen(feed().withName("Manual Feed"))
            .withName("Shoot Manually")
            .handleInterrupt(() -> shooter.stop());
    }


    public Command sweetSpot() {
        return Commands.parallel(
            hood.runOnce(() -> hood.setPosition(0.21)), // was 0.19
            shooter.spinUpCommand(4450) //was 5000rpm
        ).withName("Sweet Spot Prepare")
        .andThen(feed().withName("Sweet Spot Feed"))
        .withName("Sweet Spot")
        .finallyDo(() -> {
            shooter.stop();
            hood.setPosition(0.15);
        });
    }


    public Command snowPlow() {
        return Commands.parallel(
            hood.runOnce(() -> hood.setPosition(0.75)),
            shooter.spinUpCommand(3500), //was 4000rpm
            Commands.waitSeconds(0.4) // was 0.5, lowered to see if we can increase shooting speed - this is the time for the hood to get to position and shooter to get up to speed
                .andThen(Commands.parallel(
                    intake.intakeCommand(),
                    floor.feedCommand(),
                    feeder.feedCommand()
                ).withName("Snow Plow Feed"))
        ).withName("Snow Plow")
        .finallyDo(() -> {
            shooter.stop();
            hood.setPosition(0.15);
        });
    }

    public Command hubShot() {
        return Commands.parallel(
            hood.runOnce(() -> hood.setPosition(0.0)),
            shooter.spinUpCommand(3650) //good value now! was 3750 and very high % in the hub!
        ).withName("Hub Shot Prepare")
        .andThen(feed().withName("Hub Shot Feed"))
        .withName("Hub Shot")
        .finallyDo(() -> {
            shooter.stop();
            hood.setPosition(0.15);
        });
    }

    public Command hubShotAuton() {
        return Commands.parallel(
            hood.runOnce(() -> hood.setPosition(0.0)),
            shooter.spinUpCommand(3650) //was 3750 and very high % in the hub!
        ).withName("Hub Shot Auton Prepare")
        .andThen(feedAuton().withName("Hub Shot Auton Feed"))
        .withName("Hub Shot Auton")
        .finallyDo(() -> {
            shooter.stop();
            hood.setPosition(0.15);
        });
    }

    public Command trenchShotAuton() {
        return Commands.parallel(
            hood.runOnce(() -> hood.setPosition(0.15)),
            shooter.spinUpCommand(3800) // 4000 was perfect, but I moved closer to hub by about 6 inches
        ).withName("Trench Shot Auton Prepare")
        .andThen(feedAuton().withName("Trench Shot Auton Feed"))
        .withName("Trench Shot Auton")
        .finallyDo(() -> {
            shooter.stop();
            hood.setPosition(0.15);
        });
    }


    public Command reverseDeliver() {
        return Commands.parallel(
            shooter.reverseCommand(),
            feeder.reverseFeedCommand(),
            floor.reverseFeedCommand()
        ).withName("Reverse Deliver");
    }

    private Command feed() {
        return Commands.sequence(
            Commands.waitSeconds(0.25), // was .25, lowered to see if we can increase shooting speed
            Commands.parallel(
                feeder.feedCommand(),
                Commands.waitSeconds(1.00) // was .125
                    .andThen(floor.feedCommand().alongWith(intake.agitateCommand()).withName("FeedAndAgitate"))
            )
        );
    }
    // Brief reverse of feeder and floor to prevent jamming on release
    public Command briefReverse() {
        return Commands.parallel(
            feeder.reverseFeedCommand(),
            floor.reverseFeedCommand()
        ).withName("Brief Reverse").withTimeout(0.18);
    }

    //No agitation for auton feed to prevent jamming on the way out of the hub
    private Command feedAuton() {
        return Commands.sequence(
            Commands.waitSeconds(0.25),
            Commands.parallel(
                feeder.feedCommand(),
                Commands.waitSeconds(0.125)
                    .andThen(floor.feedCommand())
            )
        );
    }
}

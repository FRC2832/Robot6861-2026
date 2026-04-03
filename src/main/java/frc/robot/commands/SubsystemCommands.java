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
    private final double maxSpeed;
    private final double maxAngularRate;
    private final DoubleSupplier speedMultiplierSupplier;

    public SubsystemCommands(
        CommandSwerveDrivetrain swerve,
        IntakeSubsystem intake,
        FloorSubsystem floor,
        FeederSubsystem feeder,
        ShooterSubsystem shooter,
        HoodSubsystem hood,
        HangerSubsystem hanger,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput,
        double maxSpeed,
        double maxAngularRate,
        DoubleSupplier speedMultiplierSupplier
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
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;
        this.speedMultiplierSupplier = speedMultiplierSupplier;
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
            () -> 0,
            0,
            0,
            () -> 1.0
        );
    }

    // public Command aimAndShoot() {
    //     final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
    //     final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);
    //     return Commands.parallel(
    //         aimAndDriveCommand,
    //         Commands.waitSeconds(0.25)
    //             .andThen(prepareShotCommand),
    //         Commands.waitUntil(() -> aimAndDriveCommand.isAimed() && prepareShotCommand.isReadyToShoot())
    //             .andThen(feed().withName("Aim Feed"))
    //     ).withName("Aim And Shoot");
    // }

    // public Command visionShoot() {
    //     final VisionShotCommand visionShotCommand = new VisionShotCommand(shooter, hood);
    //     return Commands.parallel(
    //         visionShotCommand,
    //         Commands.waitUntil(visionShotCommand::isReadyToShoot)
    //             .andThen(feed().withName("Vision Feed"))
    //     ).withName("Vision Shoot");
    // }

    public Command visionAlignAndShootAuton() {
        final AutoVisionAlignCommand alignCommand = new AutoVisionAlignCommand(swerve);
        final VisionShotCommand visionShotCommand = new VisionShotCommand(shooter, hood);
        return Commands.parallel(
            // Start spinning up RPM + hood while aligning
            visionShotCommand,
            Commands.sequence(
                // Align with timeout — shoots even if not perfectly aligned
                alignCommand.withTimeout(.2), //was 1.5
                Commands.waitUntil(visionShotCommand::isReadyToShoot),
                feed().withName("Vision Auton Feed")
            )
        ).withName("Vision Align And Shoot Auton");
    }

    public Command visionAimAndShoot() {
        final VisionAimCommand visionAimCommand = new VisionAimCommand(
            swerve, forwardInput, leftInput, maxSpeed, maxAngularRate, speedMultiplierSupplier);
        final VisionShotCommand visionShotCommand = new VisionShotCommand(shooter, hood);
        return Commands.parallel(
            visionAimCommand,
            visionShotCommand,
            Commands.waitUntil(() -> visionAimCommand.isAimed() && visionShotCommand.isReadyToShoot())
                .andThen(feed().withName("Vision Feed"))
        ).withName("Vision Aim And Shoot");
    }

    public Command shootManually() {
        return Commands.parallel(
            hood.runOnce(() -> hood.setPosition(0.3)), // was 0.19
            shooter.spinUpCommand(5200) //was 5650 rpm. max rpm. Takes too long to ramp up to 5900 rpm.
        )
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


    public Command snowPlowFar() {
        return Commands.sequence(
            hood.runOnce(() -> hood.setPosition(0.75)),
            Commands.waitUntil(() -> hood.getCurrentPosition() > 0.5),
            shooter.spinUpCommand(5200), //was 5600 rpm
            Commands.parallel(
                feeder.feedCommand(),
                floor.feedCommand(),
                intake.intakeCommand()
            ).withName("Snow Plow Far Feed")
        ).withName("Snow Plow Far")
        .finallyDo(() -> {
            shooter.stop();
            hood.setPosition(0.15);
        });
    }


    public Command snowPlowNear() {
        return Commands.sequence(
            hood.runOnce(() -> hood.setPosition(0.75)),
            Commands.waitUntil(() -> hood.getCurrentPosition() > 0.5),
            shooter.spinUpCommand(4000), //was 4000rpm
            Commands.parallel(
                feeder.feedCommand(),
                floor.feedCommand(),
                intake.intakeCommand()
            ).withName("Snow Plow Near Feed")
        ).withName("Snow Plow Near")
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
            Commands.waitSeconds(0.20), // was .25, lowered to see if we can increase shooting speed
            Commands.parallel(
                feeder.feedCommand(),
                Commands.waitSeconds(0.25) // was .125, was 1.0
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

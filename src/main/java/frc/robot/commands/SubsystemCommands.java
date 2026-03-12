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
                .andThen(feed())
        );
    }

    public Command shootManually() {
        return shooter.dashboardSpinUpCommand()
            .andThen(feed())
            .handleInterrupt(() -> shooter.stop());
    }


    public Command sweetSpot() {
        return Commands.parallel(
            hood.runOnce(() -> hood.setPosition(0.21)), // was 0.19
            shooter.spinUpCommand(4750) //was 5000rpm
        )
        .andThen(feed())
        .handleInterrupt(() -> shooter.stop());
    }


     public Command hubShot() {
        return Commands.parallel(
            hood.runOnce(() -> hood.setPosition(0.0)), 
            shooter.spinUpCommand(3650) //was 3750 and very high % in the hub!
        )
        .andThen(feed())
        .handleInterrupt(() -> shooter.stop());
    }



    public Command reverseDeliver() {
        return Commands.parallel(
            shooter.reverseCommand(),
            feeder.reverseFeedCommand(),
            floor.reverseFeedCommand()
        );
    }

    private Command feed() {
        return Commands.sequence(
            Commands.waitSeconds(0.25),
            Commands.parallel(
                feeder.feedCommand(),
                Commands.waitSeconds(0.125)
                    .andThen(floor.feedCommand().alongWith(intake.agitateCommand()))
            )
        );
    }
}

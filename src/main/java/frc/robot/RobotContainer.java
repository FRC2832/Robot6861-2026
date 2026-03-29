// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.HeadingLockDriveCommand;
import frc.robot.commands.SpeedModeCMD;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.util.DriveInputSmoother;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;

public class RobotContainer {
    // TODO: Once robot has bumpers and drivers are able to drive proficiently increase drivespeedreduction
    private final double MaxSpeed = Constants.Driving.DriveSpeedReduction * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    
    // TODO: Increase values eventually and put these values into Constants file
    private final double MaxAngularRate = Constants.Driving.AngularRateReduction * RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double speedMultiplier = 1.0;

    // Slew rate limiters — strafe more restrictive than forward/back
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);   // TODO: tune — forward/back
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(2.0);   // TODO: tune — strafe (more limiting)
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0); // TODO: tune — rotation

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private UsbCamera driverCam;

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Subsystem instantiation
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    private final FloorSubsystem floorSubsystem = new FloorSubsystem();
    private final HangerSubsystem hangerSubsystem = new HangerSubsystem();
    private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem("limelight-dino");
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    // Command instantiation
    private final SubsystemCommands subsystemCommands = new SubsystemCommands(drivetrain, intakeSubsystem, floorSubsystem, feederSubsystem, shooterSubsystem, hoodSubsystem, hangerSubsystem);
    private final AutoRoutines autoRoutines = new AutoRoutines(
        drivetrain, intakeSubsystem, floorSubsystem, feederSubsystem,
        shooterSubsystem, hoodSubsystem, hangerSubsystem, limelightSubsystem
    );

    public RobotContainer() {
        configureBindings();
        autoRoutines.configure();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // Original default command (no heading correction — drifts when strafing)
        // drivetrain.setDefaultCommand(
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(xLimiter.calculate(driverController.getLeftY()) * MaxSpeed * speedMultiplier)
        //             .withVelocityY(yLimiter.calculate(driverController.getLeftX()) * MaxSpeed * speedMultiplier)
        //             .withRotationalRate(rotLimiter.calculate(-driverController.getRightX()) * MaxAngularRate * speedMultiplier)
        //             .withDeadband(MaxSpeed * speedMultiplier * 0.15)
        //     )
        // );

        // Heading-lock drive: holds heading via PID when rotation stick is idle
        final HeadingLockDriveCommand headingLockDrive = new HeadingLockDriveCommand(
            drivetrain,
            new DriveInputSmoother(
                driverController::getLeftY,
                driverController::getLeftX,
                () -> -driverController.getRightX()
            ),
            MaxSpeed,
            MaxAngularRate,
            this::getSpeedMultiplier
        );
        
        drivetrain.setDefaultCommand(headingLockDrive);

        //Limelight disabled to reduce CPU usage
        //limelightSubsystem.setDefaultCommand(
        //    limelightSubsystem.run(() -> {
        //     var measurement = limelightSubsystem.getMeasurement(drivetrain.getState().Pose);
        //     measurement.ifPresent(m ->
        //         drivetrain.addVisionMeasurement(
        //            m.poseEstimate.pose,
        //            m.poseEstimate.timestampSeconds,
        //            m.standardDeviations
        //             )
        //         );
        //     })
        //     .ignoringDisable(true)
        // );

        intakeSubsystem.setDefaultCommand(intakeSubsystem.run(() -> intakeSubsystem.set(IntakeSubsystem.Speed.STOP)).withName("IntakeIdle"));
        
        shooterSubsystem.setDefaultCommand(shooterSubsystem.idleCommand());

        hoodSubsystem.setDefaultCommand(
            hoodSubsystem.run(() -> {
                double joystickY = -operatorController.getRightY();
                if (Math.abs(joystickY) > 0.1) {
                    hoodSubsystem.setPosition(hoodSubsystem.targetPosition + joystickY *0.005);
                }  
            }
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // DISABLED: USB camera on roboRIO drew too much current, causing the roboRIO to brown out and shut down the robot.
        // Use PhotonVision on a coprocessor instead for driver camera.
        //driverCam = CameraServer.startAutomaticCapture("Driver Cam", 0);
        //driverCam.setResolution(640, 480);
        //driverCam.setFPS(20);


        driverController.y().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on start press, and re-sync heading lock target.
        driverController.start().onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldCentric();
            headingLockDrive.resetTargetHeading();
        }));

        // Back button disables shooter idle (pit safety)
        driverController.back().onTrue(shooterSubsystem.runOnce(() -> shooterSubsystem.setIdleEnabled(false)));

        //this is turtle mode //TODO: decrease thresholds as robot speed increases
        driverController.leftTrigger(0.2).whileTrue(new SpeedModeCMD(this,0.7));
        
        //this is snail mode //TODO: decrease thresholds as robot speed increases
        driverController.leftTrigger(0.6).whileTrue(new SpeedModeCMD(this,0.4));

        drivetrain.registerTelemetry(logger::telemeterize);


        // Set to a button for homing in the pit
        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
             .onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.seedPosition(65)))
             .onTrue(shooterSubsystem.runOnce(() -> shooterSubsystem.setIdleEnabled(true)));
            
           // .onTrue(intakeSubsystem.homingCommand())
           // .onTrue(hangerSubsystem.homingCommand());

        // Aim and shoot disabled - turns wrong direction - might be due to Pigeon orientation
        driverController.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
        driverController.x().whileTrue(subsystemCommands.shootManually())
            .onFalse(subsystemCommands.briefReverse());

        //Sweet Spot
        driverController.rightBumper().whileTrue(subsystemCommands.sweetSpot())
            .onFalse(subsystemCommands.briefReverse());

         //Hub Shot
        driverController.leftBumper().whileTrue(subsystemCommands.hubShot())
            .onFalse(subsystemCommands.briefReverse());

        
        
        //CLimbing mechanism
        //TODO: uncomment for hanging when climber meets perimeter rules  
        //driverController.povUp().onTrue(hangerSubsystem.positionCommand(HangerSubsystem.Position.HANGING));
        driverController.povUp().onTrue(hangerSubsystem.positionCommand(HangerSubsystem.Position.EXTEND_HOPPER));
        
        driverController.povDown().onTrue(hangerSubsystem.positionCommand(HangerSubsystem.Position.HUNG));


        // Intake controls
        operatorController.rightTrigger().whileTrue(
            intakeSubsystem.intakeCommand().alongWith(floorSubsystem.feedCommand())
        ); // was leftTrigger, added floor feed
        operatorController.leftTrigger().whileTrue(intakeSubsystem.reverseIntakeCommand());

        operatorController.start().onTrue(intakeSubsystem.stowCommand());
        operatorController.back().onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.runOnce(() -> intakeSubsystem.seedPosition(0)))); // was leftBumper
        
        // Shooter controls
        operatorController.b().onTrue(hoodSubsystem.runOnce(() -> hoodSubsystem.setPosition(0.158)));
        operatorController.rightBumper().whileTrue(subsystemCommands.shootManually())
            .onFalse(subsystemCommands.briefReverse());

        // Sweet Spot
        operatorController.x().whileTrue(subsystemCommands.sweetSpot())
            .onFalse(subsystemCommands.briefReverse());
        operatorController.leftBumper().whileTrue(subsystemCommands.reverseDeliver());

        // Snowplow
        operatorController.y().whileTrue(subsystemCommands.snowPlow());
        

        // Agitate command
        operatorController.a().whileTrue(intakeSubsystem.agitateCommand());

        // Shooter RPM tuning
        operatorController.povUp().onTrue(shooterSubsystem.runOnce(shooterSubsystem::incrementTargetRPM));
        operatorController.povDown().onTrue(shooterSubsystem.runOnce(shooterSubsystem::decrementTargetRPM));

        // Emergency CANivore USB reset.  this didn't fix our issue. LEaving it in for now. CTRE id 29ca-
        //operatorController.y().onTrue(Commands.runOnce(() -> {
           // try {
           //     Runtime.getRuntime().exec(new String[]{"usbreset", "/dev/bus/usb/001/003"});
           // } catch (Exception e) {
           //     e.printStackTrace();
           //}
        //}).ignoringDisable(true));  
        
    }

    // THIS METHOD IS COPIED FROM WCP
    /* 
    private void configureManualDriveBindings() {
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swerve, 
            () -> -driver.getLeftY(), 
            () -> -driver.getLeftX(), 
            () -> -driver.getRightX()
        );
        swerve.setDefaultCommand(manualDriveCommand);
        driver.a().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.k180deg)));
        driver.b().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCW_90deg)));
        driver.x().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCCW_90deg)));
        driver.y().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kZero)));
        driver.back().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric()));
    }
    */

    // Autonomous is now handled by AutoRoutines and the AutoChooser on SmartDashboard
    /*
    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
    */

    public void setSpeedMultiplier(double multiplier) {
        this.speedMultiplier = multiplier;
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }
}

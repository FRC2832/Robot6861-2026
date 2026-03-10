// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.commands.SpeedModeCMD;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem("limelight");
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    // Command instantiation
    private final SubsystemCommands subsystemCommands = new SubsystemCommands(drivetrain, intakeSubsystem, floorSubsystem, feederSubsystem, shooterSubsystem, hoodSubsystem, hangerSubsystem);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driverController.getLeftY() * MaxSpeed * speedMultiplier) // Drive forward with negative Y (forward)
                    .withVelocityY(driverController.getLeftX() * MaxSpeed * speedMultiplier) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate * speedMultiplier) // Drive counterclockwise with negative X (left)
                    .withDeadband(MaxSpeed * speedMultiplier * 0.15)
            )
        );

        // limelightSubsystem.setDefaultCommand(
           // limelightSubsystem.run(() -> {
             //   var measurement = limelightSubsystem.getMeasurement(drivetrain.getState().Pose);
               // measurement.ifPresent(m ->
                 //   drivetrain.addVisionMeasurement(
                   //     m.poseEstimate.pose,
                     //   m.poseEstimate.timestampSeconds,
                       // m.standardDeviations
                    //)
               // );
           // })
        //);

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

        // Boilerplate code to start the camera server
        
        driverCam = CameraServer.startAutomaticCapture("Driver Cam", 0);
        driverCam.setResolution(640, 480);
        driverCam.setFPS(20);


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

        // Reset the field-centric heading on start press.
        driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        //this is turtle mode //TODO: decrease thresholds as robot speed increases
        driverController.leftTrigger(0.2).whileTrue(new SpeedModeCMD(this,0.7));
        
        //this is snail mode //TODO: decrease thresholds as robot speed increases
        driverController.leftTrigger(0.6).whileTrue(new SpeedModeCMD(this,0.4));

        drivetrain.registerTelemetry(logger::telemeterize);


        // Set to a button for homing in the pit
        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
             .onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.seedPosition(65)));
            
           // .onTrue(intakeSubsystem.homingCommand())
           // .onTrue(hangerSubsystem.homingCommand());

        driverController.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
        driverController.rightBumper().whileTrue(subsystemCommands.shootManually());

        //Sweet Spot
        driverController.x().whileTrue(subsystemCommands.sweetSpot());
        
        driverController.povUp().onTrue(hangerSubsystem.positionCommand(HangerSubsystem.Position.HANGING));
        driverController.povDown().onTrue(hangerSubsystem.positionCommand(HangerSubsystem.Position.HUNG));


        // Intake controls
        operatorController.rightTrigger().whileTrue(intakeSubsystem.intakeCommand()); // was leftTrigger
        operatorController.start().onTrue(intakeSubsystem.homingCommand());
        operatorController.back().onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.set(IntakeSubsystem.Position.STOWED))); // was leftBumper
        
        // Shooter controls
        operatorController.b().onTrue(hoodSubsystem.runOnce(() -> hoodSubsystem.setPosition(0.158)));
        operatorController.rightBumper().whileTrue(subsystemCommands.shootManually());

        //Sweet Spot
        operatorController.x().whileTrue(subsystemCommands.sweetSpot());
        operatorController.leftBumper().whileTrue(subsystemCommands.reverseDeliver());
        

        operatorController.a().whileTrue(intakeSubsystem.agitateCommand());

        // Shooter RPM tuning
        operatorController.povUp().onTrue(shooterSubsystem.runOnce(shooterSubsystem::incrementTargetRPM));
        operatorController.povDown().onTrue(shooterSubsystem.runOnce(shooterSubsystem::decrementTargetRPM));  
        
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

    public void setSpeedMultiplier(double multiplier) {
        this.speedMultiplier = multiplier;
        
    }
}

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.util.DriveInputSmoother;
import frc.util.ManualDriveInput;

/**
 * Default drive command with automatic heading lock.
 * When the driver commands rotation, it behaves like normal open-loop driving.
 * When the driver releases the rotation stick, it captures the current heading
 * and uses a PID to hold it — preventing drift during strafing.
 */
public class HeadingLockDriveCommand extends Command {
    private static final double kRotationInputThreshold = 0.05;

    private final CommandSwerveDrivetrain swerve;
    private final DriveInputSmoother inputSmoother;
    private final double maxSpeed;
    private final double maxAngularRate;
    private final DoubleSupplier speedMultiplierSupplier;

    // Same slew rate limiters as original default command
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(2.0);

    // Used when rotation stick is idle — PID holds heading
    private final SwerveRequest.FieldCentricFacingAngle lockedRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withRotationalDeadband(Driving.kPIDRotationDeadband)
        .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
        .withHeadingPID(5, 0, 0.2);

    // Used when driver is actively rotating — same as original default command
    private final SwerveRequest.FieldCentric openRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private Rotation2d targetHeading = new Rotation2d();
    private boolean wasRotating = false;
    private boolean needsHeadingCapture = true;

    public HeadingLockDriveCommand(
        CommandSwerveDrivetrain swerve,
        DriveInputSmoother inputSmoother,
        double maxSpeed,
        double maxAngularRate,
        DoubleSupplier speedMultiplierSupplier
    ) {
        this.swerve = swerve;
        this.inputSmoother = inputSmoother;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;
        this.speedMultiplierSupplier = speedMultiplierSupplier;
        addRequirements(swerve);
    }

    /** Call this after seedFieldCentric to re-sync the target heading. */
    public void resetTargetHeading() {
        needsHeadingCapture = true;
    }

    @Override
    public void initialize() {
        targetHeading = swerve.getState().Pose.getRotation();
        wasRotating = false;
        needsHeadingCapture = false;
    }

    @Override
    public void execute() {
        ManualDriveInput input = inputSmoother.getSmoothedInput();
        double multiplier = speedMultiplierSupplier.getAsDouble();

        double vx = xLimiter.calculate(input.forward) * maxSpeed * multiplier;
        double vy = yLimiter.calculate(input.left) * maxSpeed * multiplier;
        double deadband = maxSpeed * multiplier * 0.15;

        boolean isRotating = Math.abs(input.rotation) > kRotationInputThreshold;

        if (isRotating) {
            // Driver is commanding rotation — normal open-loop, same as original
            swerve.setControl(
                openRequest
                    .withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(input.rotation * maxAngularRate * multiplier)
                    .withDeadband(deadband)
            );
            wasRotating = true;
        } else {
            // No rotation input — lock heading to prevent drift
            if (wasRotating || needsHeadingCapture) {
                // Capture current heading: after releasing stick, first strafe, or seedFieldCentric
                targetHeading = swerve.getState().Pose.getRotation();
                wasRotating = false;
                needsHeadingCapture = false;
            }
            swerve.setControl(
                lockedRequest
                    .withVelocityX(vx)
                    .withVelocityY(vy)
                    .withTargetDirection(targetHeading)
                    .withDeadband(deadband)
            );
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

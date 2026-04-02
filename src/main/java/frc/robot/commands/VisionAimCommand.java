package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.util.DriveInputSmoother;
import frc.util.ManualDriveInput;

/**
 * Lets the driver control translation while vision auto-corrects rotation
 * using Limelight tx. If no target is visible, driver has full control
 * with no rotation correction applied.
 */
public class VisionAimCommand extends Command {
    private static final String kLimelightName = "limelight-dino";

    // Deadband: if tx is within this many degrees, don't correct
    private static final double kTxDeadband = 2.0;
    // Proportional gain: how aggressively to turn toward target
    private static final double kAimP = 0.02;
    // Aim tolerance for "aimed" check (degrees)
    private static final double kAimTolerance = 5.0;

    private final CommandSwerveDrivetrain swerve;
    private final DriveInputSmoother inputSmoother;
    private final double maxSpeed;
    private final double maxAngularRate;
    private final DoubleSupplier speedMultiplierSupplier;

    // Match HeadingLockDriveCommand slew rates so driving feels the same
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(2.0);

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public VisionAimCommand(
        CommandSwerveDrivetrain swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput,
        double maxSpeed,
        double maxAngularRate,
        DoubleSupplier speedMultiplierSupplier
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;
        this.speedMultiplierSupplier = speedMultiplierSupplier;
        addRequirements(swerve);
    }

    public boolean isAimed() {
        return LimelightHelpers.getTV(kLimelightName)
            && Math.abs(LimelightHelpers.getTX(kLimelightName)) < kAimTolerance;
    }

    @Override
    public void execute() {
        ManualDriveInput input = inputSmoother.getSmoothedInput();
        double multiplier = speedMultiplierSupplier.getAsDouble();

        double vx = xLimiter.calculate(input.forward) * maxSpeed * multiplier;
        double vy = yLimiter.calculate(input.left) * maxSpeed * multiplier;
        double deadband = maxSpeed * multiplier * 0.15;

        boolean hasTarget = LimelightHelpers.getTV(kLimelightName);
        double tx = LimelightHelpers.getTX(kLimelightName);
        double rotationRate;

        if (hasTarget && Math.abs(tx) > kTxDeadband) {
            // tx positive = target right of crosshair → turn right (negative in WPILib)
            rotationRate = -tx * kAimP * maxAngularRate;
        } else {
            rotationRate = 0.0;
        }

        SmartDashboard.putBoolean("VisionAim/HasTarget", hasTarget);
        SmartDashboard.putNumber("VisionAim/tx", tx);
        SmartDashboard.putNumber("VisionAim/RotationRate", rotationRate);

        swerve.setControl(
            driveRequest
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(rotationRate)
                .withDeadband(deadband)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

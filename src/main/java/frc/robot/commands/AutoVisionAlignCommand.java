package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Autonomous-only command that rotates the robot until the Limelight target
 * is centered (tx near zero). Finishes automatically when aligned.
 * No translation — robot stays in place and just rotates.
 */
public class AutoVisionAlignCommand extends Command {
    private static final String kLimelightName = "limelight-dino";

    // How close tx must be to zero to consider aligned (degrees)
    private static final double kAlignTolerance = 3.0; //was 3.0
    // Deadband: don't correct below this (degrees)
    private static final double kTxDeadband = 2.0; //on May 4, test with 1.75.Need AdvnatageScope data to look at rotation. was 2.0 & 1.0 before that
    // Proportional gain for rotation
    private static final double kAimP = 0.04;
    // Max rotation speed (rad/s) — keep moderate for auton safety
    private static final double kMaxRotationRate = 1.5;

    private static final int kBlueTagID = 25;
    private static final int kRedTagID = 9;

    private final CommandSwerveDrivetrain swerve;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AutoVisionAlignCommand(CommandSwerveDrivetrain swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // Lock onto the correct hub tag for our alliance side
        Optional<Alliance> alliance = DriverStation.getAlliance();
        int tagID = (alliance.isPresent() && alliance.get() == Alliance.Blue) ? kBlueTagID : kRedTagID;
        LimelightHelpers.setPriorityTagID(kLimelightName, tagID);
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV(kLimelightName);
        double tx = LimelightHelpers.getTX(kLimelightName);

        SmartDashboard.putBoolean("AutoAlign/HasTarget", hasTarget);
        SmartDashboard.putNumber("AutoAlign/tx", tx);

        double rotationRate = 0.0;
        if (hasTarget && Math.abs(tx) > kTxDeadband) {
            // tx positive = target right of crosshair → turn right (negative in WPILib)
            rotationRate = -tx * kAimP * kMaxRotationRate;
        }

        swerve.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotationRate)
        );
    }

    @Override
    public boolean isFinished() {
        boolean hasTarget = LimelightHelpers.getTV(kLimelightName);
        double tx = LimelightHelpers.getTX(kLimelightName);
        return hasTarget && Math.abs(tx) < kAlignTolerance;
    }
}

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
import frc.robot.Landmarks;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.util.DriveInputSmoother;
import frc.util.GeometryUtil;
import frc.util.ManualDriveInput;

public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);

    private final CommandSwerveDrivetrain swerve;
    private final DriveInputSmoother inputSmoother;

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(Driving.kPIDRotationDeadband)
        .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(2.0, 0, 0); //was 5, lowered for debugging

    public AimAndDriveCommand(
        CommandSwerveDrivetrain swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        addRequirements(swerve);
    }

    public AimAndDriveCommand(CommandSwerveDrivetrain swerve) {
        this(swerve, () -> 0, () -> 0);
    }

    public boolean isAimed() {
        final Rotation2d targetHeading = fieldCentricFacingAngleRequest.TargetDirection;
        final Rotation2d currentHeadingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        return GeometryUtil.isNear(targetHeading, currentHeadingInOperatorPerspective, kAimTolerance);
    }

    private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        //final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
        //final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        //return hubDirectionInOperatorPerspective;

        return hubPosition.minus(robotPosition).getAngle(); // no rotateBy here
    }

    @Override
    public void execute() {
        final ManualDriveInput input = inputSmoother.getSmoothedInput();

        final Rotation2d targetHeading = getDirectionToHub();
        final Rotation2d currentHeading = swerve.getState().Pose.getRotation();
        final Rotation2d operatorForward = swerve.getOperatorForwardDirection();
        final double headingErrorDeg = targetHeading.minus(currentHeading).getDegrees();

        //SignalLogger.writeDouble("Aim/TargetHeadingDeg", targetHeading.getDegrees());
        //SignalLogger.writeDouble("Aim/CurrentHeadingDeg", currentHeading.getDegrees());
        //SignalLogger.writeDouble("Aim/OperatorForwardDeg", operatorForward.getDegrees());
        //SignalLogger.writeDouble("Aim/HeadingErrorDeg", headingErrorDeg);
        //SignalLogger.writeDouble("Aim/CommandedForward", input.forward);
        //SignalLogger.writeDouble("Aim/CommandedLeft", input.left);

        SmartDashboard.putNumber("Aim/TargetHeadingDeg", targetHeading.getDegrees());
        SmartDashboard.putNumber("Aim/CurrentHeadingDeg", currentHeading.getDegrees());
        SmartDashboard.putNumber("Aim/OperatorForwardDeg", operatorForward.getDegrees());
        SmartDashboard.putNumber("Aim/HeadingErrorDeg", headingErrorDeg);
        SmartDashboard.putNumber("Aim/CommandedForward", input.forward);
        SmartDashboard.putNumber("Aim/CommandedLeft", input.left);


        swerve.setControl(
            fieldCentricFacingAngleRequest
                .withVelocityX(Driving.kMaxSpeed.times(input.forward))
                .withVelocityY(Driving.kMaxSpeed.times(input.left))
                .withTargetDirection(getDirectionToHub())
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

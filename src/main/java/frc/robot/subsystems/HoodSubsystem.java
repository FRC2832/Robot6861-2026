package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class HoodSubsystem extends SubsystemBase {
    private static final Distance kServoLength = Millimeters.of(100);
    private static final LinearVelocity kMaxServoSpeed = Millimeters.of(20).per(Second);
    private static final double kMinPosition = 0.01;
    private static final double kMaxPosition = 0.77;
    private static final double kPositionTolerance = 0.01;

    private final ServoHub servoHub;
    private final ServoChannel leftServo;
    private final ServoChannel rightServo;

    private double currentPosition = 0.01;
    public double targetPosition = 0.5;
    private Time lastUpdateTime = Seconds.of(0);

   

    public HoodSubsystem() {
        servoHub = new ServoHub(Ports.kHoodServoHub);

        ServoHubConfig config = new ServoHubConfig();
        config.channel0.pulseRange(1000, 1500, 2000);
        config.channel1.pulseRange(1000, 1500, 2000);
        servoHub.configure(config, ResetMode.kResetSafeParameters);

        leftServo = servoHub.getServoChannel(ChannelId.kChannelId0);
        rightServo = servoHub.getServoChannel(ChannelId.kChannelId1);
        //leftServo.setPulseWidth(1000);
        //rightServo.setPulseWidth(1000);
        leftServo.setEnabled(true);
        rightServo.setEnabled(true);
        rightServo.setPowered(true);
        leftServo.setPowered(true);
        setPosition(currentPosition);
        SmartDashboard.putData(this);
    }

    public void setDefaultCommand(XboxController operatorController){
        setDefaultCommand(
            run(() -> {
                double joystickY = -operatorController.getRightY();
                if (Math.abs(joystickY) > 0.1) {
                    this.setPosition(targetPosition + joystickY * 0.0085); //was 0.005
                }
            })
        );
    }

    /** Expects a position between 0.0 and 1.0 */
    public void setPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);
          // ServoChannel uses microseconds, convert 0-1 range to pulse width
          int pulseWidth = (int)(1000 + clampedPosition * 1000);
          leftServo.setPulseWidth(pulseWidth);
          rightServo.setPulseWidth(pulseWidth);
          targetPosition = clampedPosition;
    }

    /** Expects a position between 0.0 and 1.0 */
    public Command positionCommand(double position) {
        return runOnce(() -> setPosition(position))
            .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }

    public double getCurrentPosition() {
        return currentPosition;
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, currentPosition, kPositionTolerance);
    }

    private void updateCurrentPosition() {
        final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
        final Time elapsedTime = currentTime.minus(lastUpdateTime);
        lastUpdateTime = currentTime;

        if (isPositionWithinTolerance()) {
            currentPosition = targetPosition;
            return;
        }

        final Distance maxDistanceTraveled = kMaxServoSpeed.times(elapsedTime);
        final double maxPercentageTraveled = maxDistanceTraveled.div(kServoLength).in(Value);
        currentPosition = targetPosition > currentPosition
            ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
            : Math.max(targetPosition, currentPosition - maxPercentageTraveled);
    }

    @Override
    public void periodic() {
        updateCurrentPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Hood Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Hood Current Position", () -> currentPosition, null);
        builder.addDoubleProperty("Hood Target Position", () -> targetPosition, value -> setPosition(value));
    }
}
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class ShooterSubsystem extends SubsystemBase {
    private static final AngularVelocity kVelocityTolerance = RPM.of(100);

    private final TalonFX leftMotor, middleMotor, rightMotor;
    private final List<TalonFX> motors;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private double dashboardTargetRPM = 5000.0; // was 4000.0
    private double idleRPM = 1500.0;
    private double rpmStep = 100.0;
    private boolean idleEnabled = true;
    private double kA = 0.06; 

    public ShooterSubsystem() {
        leftMotor = new TalonFX(Ports.kShooterLeft, Ports.kCANivoreCANBus);
        middleMotor = new TalonFX(Ports.kShooterMiddle, Ports.kCANivoreCANBus);
        rightMotor = new TalonFX(Ports.kShooterRight, Ports.kCANivoreCANBus);
        motors = List.of(leftMotor, middleMotor, rightMotor);

        // Confirm appropriate inversion, voltage limits, current limits, and PID constants
        configureMotor(leftMotor, InvertedValue.CounterClockwise_Positive);
        configureMotor(middleMotor, InvertedValue.CounterClockwise_Positive); // inverted
        configureMotor(rightMotor, InvertedValue.Clockwise_Positive);

        SmartDashboard.putData(this);
    }

    private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakReverseVoltage(Volts.of(0))
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(90))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.5)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
                    .withKA(0.0) // could be tuned to improve response, but leaving at 0 for now since velocity loop is already pretty aggressive and we want to avoid overshoot
            );
        
        motor.getConfigurator().apply(config);
    }

    public void setRPM(double rpm) {
        for (final TalonFX motor : motors) {
            motor.setControl(
                velocityRequest
                    .withVelocity(RPM.of(rpm))
            );
        }
    }

    public void setPercentOutput(double percentOutput) {
        for (final TalonFX motor : motors) {
            motor.setControl(
                voltageRequest
                    .withOutput(Volts.of(percentOutput * 12.0))
            );
        }
    }

    public void stop() {
        setPercentOutput(0.0);
    }

    public Command reverseCommand() {
        return startEnd(() -> setPercentOutput(-0.3), () -> stop());
    }

    public Command spinUpCommand(double rpm) {
        return runOnce(() -> setRPM(rpm))
            .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }


    public Command dashboardSpinUpCommand() {
        return defer(() -> spinUpCommand(dashboardTargetRPM));
    }

    public void setIdleEnabled(boolean enabled) {
        idleEnabled = enabled;
    }

    public Command idleCommand() {
        return run(() -> {
            if (idleEnabled) {
                setRPM(idleRPM);
            } else {
                stop();
            }
        });
    }

    private void applyKA(double newKA) {
        kA = newKA;
        final Slot0Configs slot0 = new Slot0Configs();
        for (final TalonFX motor : motors) {
            motor.getConfigurator().refresh(slot0);
            slot0.withKA(kA);
            motor.getConfigurator().apply(slot0);
        }
    }

    public void incrementTargetRPM() {
        dashboardTargetRPM += rpmStep;
    }

    public void decrementTargetRPM() {
        dashboardTargetRPM = Math.max(0, dashboardTargetRPM - rpmStep);
    }

    public boolean isVelocityWithinTolerance() {
        return motors.stream().allMatch(motor -> {
            final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
            final AngularVelocity currentVelocity = motor.getVelocity().getValue();
            final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
            return isInVelocityMode && currentVelocity.isNear(targetVelocity, kVelocityTolerance);
        });
    }

    private void initSendable(SendableBuilder builder, TalonFX motor, String name) {
        builder.addDoubleProperty(name + " SHooter RPM", () -> motor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty(name + " Shooter Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty(name + " Shooter Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty(name + " Shooter Temp (F)", () -> motor.getDeviceTemp().getValue().in(Fahrenheit), null);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        initSendable(builder, leftMotor, "Left");
        initSendable(builder, middleMotor, "Middle");
        initSendable(builder, rightMotor, "Right");
        builder.addStringProperty("Shooter Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("SHooter Dashboard RPM", () -> dashboardTargetRPM, value -> dashboardTargetRPM = value);
        builder.addDoubleProperty("SHooter Target RPM", () -> velocityRequest.getVelocityMeasure().in(RPM), null);
        builder.addDoubleProperty("Shooter Idle RPM", () -> idleRPM, value -> idleRPM = value);
        builder.addDoubleProperty("Shooter RPM Step", () -> rpmStep, value -> rpmStep = value);
        builder.addBooleanProperty("Shooter Idle Enabled", () -> idleEnabled, value -> idleEnabled = value);
        builder.addDoubleProperty("Shooter kA", () -> kA, this::applyKA);
    }
}

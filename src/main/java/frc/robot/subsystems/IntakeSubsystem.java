package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class IntakeSubsystem extends SubsystemBase {
    public enum Speed {
        STOP(0),
        INTAKE(0.60), // TODO: set up at 0.8 for comp!, 0.6 was good to keep temps down
        REVERSEINTAKE(-0.4); //was -0.3 and -0.5

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    public enum Position {
        HOMED(75),
        STOWED(65),
        // INTAKE(12.0), // was -12, -4, was -78 most recently — removed for hybrid gravity-drop approach
        // AGITATE(20),  // was 20, was -50 most recently — replaced with high agitate positions
        AGITATE_HIGH(25),  // TODO: was 30 tune — upper bound of agitate oscillation
        AGITATE_LOW(15),   // TODO: was 20 tune — lower bound of agitate oscillation
        INTAKE_DOWN(0);


        private final double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }

        public Angle angle() {
            return Degrees.of(degrees);
        }
    }

    private static final double kPivotReduction = 50.0; //was 50
    private static final AngularVelocity kMaxPivotSpeed = KrakenX60.kFreeSpeed.div(kPivotReduction);
    private static final Angle kPositionTolerance = Degrees.of(7);

    private final TalonFX pivotMotor, rollerMotor;
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0);
    private final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);

    private boolean isHomed = false;

    public IntakeSubsystem() {
        pivotMotor = new TalonFX(Ports.kIntakePivot, Ports.kCANivoreCANBus);
        rollerMotor = new TalonFX(Ports.kIntakeRollers, Ports.kCANivoreCANBus);
        configurePivotMotor();
        configureRollerMotor();
        SmartDashboard.putData(this);
    }

    private void configurePivotMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(60))  //was 90
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(20))  //was 60
                    .withSupplyCurrentLimitEnable(true)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(kPivotReduction)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(kMaxPivotSpeed)
                    .withMotionMagicAcceleration(kMaxPivotSpeed.per(Second))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(150) //was 300
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / kMaxPivotSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        pivotMotor.getConfigurator().apply(config);
    }

    private void configureRollerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(60)) //was 70a
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(50)) //was 60a
                    .withSupplyCurrentLimitEnable(true)
            );
        rollerMotor.getConfigurator().apply(config);
    }

    private boolean isPositionWithinTolerance() {
        final Angle currentPosition = pivotMotor.getPosition().getValue();
        final Angle targetPosition = pivotMotionMagicRequest.getPositionMeasure();
        return currentPosition.isNear(targetPosition, kPositionTolerance);
    }

    private void setPivotPercentOutput(double percentOutput) {
        pivotMotor.setControl(
            pivotVoltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }

    public void set(Position position) {
        pivotMotor.setControl(
            pivotMotionMagicRequest
                .withPosition(position.angle())
        );
    }

    public void seedPosition(double degrees) {
        pivotMotor.setPosition(Degrees.of(degrees));
    }

    public void set(Speed speed) {
        rollerMotor.setControl(
            rollerVoltageRequest
                .withOutput(speed.voltage())
        );
    }

    public void stopRollers() {
        rollerMotor.setControl(new NeutralOut());
    }

    // HYBRID APPROACH: gravity-drop for intake, PID only for upward moves
    // Old PID-driven commands are commented out below each new version

    public Command deployCommand() {
        return Commands.sequence(
            runOnce(() -> {
                if (pivotMotor.getPosition().getValue().in(Degrees) > 30) {
                    setPivotPercentOutput(-0.15);
                }
            }),
            Commands.waitSeconds(1.0),
            runOnce(() -> {
                seedPosition(0);
                setPivotPercentOutput(0);
            })
        );
    }

    public Command intakeCommand() {
        return Commands.sequence(
            runOnce(() -> {
                if (pivotMotor.getPosition().getValue().in(Degrees) > 30) {
                    setPivotPercentOutput(-0.16);  // was -0.2 - too much nudge only if arm is up, skip if already down
                }
                set(Speed.INTAKE);
            }),
            Commands.waitSeconds(1.0),  // was 1.5 - too fast TODO: tune — minimum time to ensure arm settles on bumpers
            runOnce(() -> {
                seedPosition(0);         // intake is down, reset encoder reference
                setPivotPercentOutput(0); // stop nudging, arm is resting on bumpers
            }),
            run(() -> set(Speed.INTAKE))  // keep commanding rollers every cycle until trigger released
        )
        .finallyDo(() -> {
            setPivotPercentOutput(0);  // stop pivot nudge
            stopRollers();
        })
        .withName("Intake");
    }
    // OLD intakeCommand — PID drove pivot down (chain slack/skip issues):
    // public Command intakeCommand() {
    //     return startEnd(
    //         () -> {
    //             set(Position.INTAKE);
    //             set(Speed.INTAKE);
    //         },
    //         () -> set(Speed.STOP)
    //     );
    // }

    public Command reverseIntakeCommand() {
        return run(() -> {
                set(Position.INTAKE_DOWN);
                set(Speed.REVERSEINTAKE);
            })
            .finallyDo(() -> stopRollers())
            .withName("ReverseIntake");
    }
    // OLD reverseIntakeCommand:
    // public Command reverseIntakeCommand() {
    //     return startEnd(
    //         () -> {
    //             set(Position.AGITATE);
    //             set(Speed.REVERSEINTAKE);
    //         },
    //         () -> set(Speed.STOP)
    //     );
    // }

    public Command agitateCommand() {
        return runOnce(() -> set(Speed.INTAKE))
            .andThen(
                Commands.sequence(
                    runOnce(() -> set(Position.AGITATE_LOW)),
                    Commands.waitUntil(this::isPositionWithinTolerance),
                    runOnce(() -> set(Speed.INTAKE)), //tried reverseintake speed, but they went out
                   
                    runOnce(() -> set(Position.AGITATE_HIGH)),
                    Commands.waitUntil(this::isPositionWithinTolerance)
                )
                .repeatedly()
            )
            .handleInterrupt(() -> {
                setPivotPercentOutput(0);  // coast down
                stopRollers();
            })
            .withName("Agitate");
    }
    // OLD agitateCommand — oscillated in the low/chain-slack zone:
    // public Command agitateCommand() {
    //     return runOnce(() -> set(Speed.INTAKE))
    //         .andThen(
    //             Commands.sequence(
    //                 runOnce(() -> set(Position.AGITATE)),
    //                 Commands.waitUntil(this::isPositionWithinTolerance),
    //                 runOnce(() -> set(Position.INTAKE)),
    //                 Commands.waitUntil(this::isPositionWithinTolerance)
    //             )
    //             .repeatedly()
    //         )
    //         .handleInterrupt(() -> {
    //             set(Position.INTAKE);
    //             set(Speed.STOP);
    //         });
    // }

    public Command homingCommand() {
        return Commands.sequence(
            runOnce(() -> setPivotPercentOutput(0.4)), // was 0.1
            Commands.waitUntil(() -> pivotMotor.getSupplyCurrent().getValue().in(Amps) > 6),
            runOnce(() -> {
                pivotMotor.setPosition(Position.HOMED.angle());
                isHomed = true;
                set(Position.STOWED);
            })
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command stowCommand() {
        return Commands.sequence(
            runOnce(() -> setPivotPercentOutput(0.375)), // was 0.4
            Commands.waitUntil(() -> pivotMotor.getSupplyCurrent().getValue().in(Amps) > 6),
            runOnce(() -> set(Position.STOWED)),
            Commands.waitUntil(() -> pivotMotor.getPosition().getValue().in(Degrees) > (Position.STOWED.angle().in(Degrees) - 5)),
            runOnce(() -> isHomed = true)
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Intake Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Pivot Angle (degrees)", () -> pivotMotor.getPosition().getValue().in(Degrees), null);
        builder.addDoubleProperty("Pivot Speed (rpm)", () -> pivotMotor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty("Roller RPM", () -> rollerMotor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty("Pivot Supply Current", () -> pivotMotor.getSupplyCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty("Roller Supply Current", () -> rollerMotor.getSupplyCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty("Roller Stator Current", () -> rollerMotor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty("Pivot Temp (F)", () -> pivotMotor.getDeviceTemp().getValue().in(Fahrenheit), null);
        builder.addDoubleProperty("Roller Temp (F)", () -> rollerMotor.getDeviceTemp().getValue().in(Fahrenheit), null);
        builder.addDoubleProperty("Roller Applied Volts", () -> rollerMotor.getMotorVoltage().getValue().in(Volts), null);
    }
}
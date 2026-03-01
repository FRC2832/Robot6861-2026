// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    //public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("CANIvore");

    // Talon FX IDs
    public static final int kIntakePivot = 3; // Possibly Intake/Deploy
    public static final int kIntakeRollers = 5; // Indexer/Hopper
    public static final int kFloor = 4; // We think this is Intake Fuel
    public static final int kFeeder = 6; // Possibly Indexer/Shooter
    public static final int kShooterLeft = 9;
    public static final int kShooterMiddle = 8;
    public static final int kShooterRight = 7;
    public static final int kHanger = 10; // Climber

    // PWM Ports
    public static final int kHoodLeftServo = 3;
    public static final int kHoodRightServo = 4;
}
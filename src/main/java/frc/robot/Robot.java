// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Elastic;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    // Teleop phase timer - notifications at 10, 35, 60, 85, 110 seconds
    // Rumble 4 seconds before each phase change
    private final Timer teleopTimer = new Timer();
    private int nextPhaseIndex = 0;
    private int nextRumbleIndex = 0;
    private static final double[] PHASE_TIMES = {10, 35, 60, 85, 110};
    private static final double[] RUMBLE_TIMES = {6, 31, 56, 81, 106};
    private static final String[] PHASE_MESSAGES = {
        "PHASE CHANGE", "PHASE CHANGE", "PHASE CHANGE", "PHASE CHANGE", "ENDGAME"
    };
    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);
    private boolean rumbleActive = false;
    private double rumbleStartTime = 0;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        //SignalLogger.setPath("/media/sda1/ctre-logs/");
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        // Autonomous is now handled by AutoRoutines and the AutoChooser via RobotModeTriggers
        /*
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);

        // Original code - figure out why it started running
        //if (m_autonomousCommand != null) {
            //CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
        */
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        teleopTimer.reset();
        teleopTimer.start();
        nextPhaseIndex = 0;
        nextRumbleIndex = 0;
        rumbleActive = false;
    }

    @Override
    public void teleopPeriodic() {
        // Rumble 4 seconds before phase change
        if (nextRumbleIndex < RUMBLE_TIMES.length
                && teleopTimer.hasElapsed(RUMBLE_TIMES[nextRumbleIndex])) {
            driverController.setRumble(RumbleType.kBothRumble, 1.0);
            operatorController.setRumble(RumbleType.kBothRumble, 1.0);
            rumbleActive = true;
            rumbleStartTime = teleopTimer.get();
            nextRumbleIndex++;
        }

        // Stop rumble after 1 second
        if (rumbleActive && teleopTimer.get() - rumbleStartTime >= 1.0) {
            driverController.setRumble(RumbleType.kBothRumble, 0);
            operatorController.setRumble(RumbleType.kBothRumble, 0);
            rumbleActive = false;
        }

        // Send notification at phase change
        if (nextPhaseIndex < PHASE_TIMES.length
                && teleopTimer.hasElapsed(PHASE_TIMES[nextPhaseIndex])) {
            Elastic.NotificationLevel level = nextPhaseIndex == 4
                ? Elastic.NotificationLevel.ERROR
                : Elastic.NotificationLevel.WARNING;
            Elastic.sendNotification(level, PHASE_MESSAGES[nextPhaseIndex], "", 5000);
            nextPhaseIndex++;
        }
    }

    @Override
    public void teleopExit() {
        driverController.setRumble(RumbleType.kBothRumble, 0);
        operatorController.setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}

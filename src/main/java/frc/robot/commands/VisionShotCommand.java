package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class VisionShotCommand extends Command {
    private static final String kLimelightName = "limelight-dino";

    // ta (target area %) -> shooter RPM
    private static final InterpolatingDoubleTreeMap taToRPM = new InterpolatingDoubleTreeMap();
    // ta (target area %) -> hood position (0.0 - 0.77)
    private static final InterpolatingDoubleTreeMap taToHood = new InterpolatingDoubleTreeMap();

    static {
        // TODO: Fill in ta values from testing at each distance
        // Larger ta = closer to hub, smaller ta = farther away
        // ta at ~48 inches (hub shot):    taToRPM.put(??, 3650);  taToHood.put(??, 0.0);
        // ta at ~65 inches (sweet spot):  taToRPM.put(??, 4450);  taToHood.put(??, 0.21);
        // ta at ~114 inches:              taToRPM.put(??, 4000);  taToHood.put(??, 0.3);
        // ta at ~165 inches:              taToRPM.put(??, 4500);  taToHood.put(??, 0.48);

        
        taToRPM.put(2.0, 3650.0);   taToHood.put(2.0, 0.0);    // hub shot (~48 in)
        taToRPM.put(0.8, 3650.0);   taToHood.put(0.8, 0.15); 
        taToRPM.put(.45, 4000.0);   taToHood.put(0.45, 0.20);
        taToRPM.put(.35, 4100.0);   taToHood.put(0.35, 0.21);  
        taToRPM.put(.24, 4200.0);   taToHood.put(0.24, 0.21);   // sweet spot (~65 in), trench shot = 0.26
        taToRPM.put(.2, 42500.0);   taToHood.put(0.2, 0.21);
        taToRPM.put(.15, 4300.0);   taToHood.put(0.15, 0.22);
        taToRPM.put(.1, 4550.0);   taToHood.put(0.1, 0.23);
        taToRPM.put(.05, 5500.0);   taToHood.put(0.05, 0.26); //was 5000
        taToRPM.put(.00, 5600.0);   taToHood.put(0.0, 0.27);

        //trench shot auton = 0.366
        //taToRPM.put(1.2, 4000.0);   taToHood.put(1.2, 0.3);    // ~114 in
        //taToRPM.put(0.2, 4500.0);   taToHood.put(0.5, 0.48);   // ~165 in
    }

    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;

    public VisionShotCommand(ShooterSubsystem shooter, HoodSubsystem hood) {
        this.shooter = shooter;
        this.hood = hood;
        addRequirements(shooter, hood);
    }

    public boolean isReadyToShoot() {
        return LimelightHelpers.getTV(kLimelightName)
            && shooter.isVelocityWithinTolerance()
            && hood.isPositionWithinTolerance();
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV(kLimelightName);
        double ta = LimelightHelpers.getTA(kLimelightName);

        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget);
        SmartDashboard.putNumber("Vision/ta", ta);

        if (hasTarget && ta > 0.01) {
            double rpm = taToRPM.get(ta);
            double hoodPos = taToHood.get(ta);

            shooter.setRPM(rpm);
            hood.setPosition(hoodPos);

            SmartDashboard.putNumber("Vision/TargetRPM", rpm);
            SmartDashboard.putNumber("Vision/TargetHood", hoodPos);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        hood.setPosition(0.15);
    }
}

package frc.robot.subsystems;
 
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
import frc.robot.LimelightHelpers;
 
public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterRightC;
    private final TalonFX shooterLeftCC;
    private final TalonFX intakeMotor;
    private final DutyCycleOut topRequest;
    private final DutyCycleOut bottomRequest;
    private final TalonFXConfiguration configA;
    private final TalonFXConfiguration configB;
 
    // ── TY to flywheel speed lookup table ─────────────────────────────────────
    // Same TY values as hood table — stand at each distance, read TY on Elastic,
    // find what flywheel speed scores, and fill in the corresponding values.
    // Higher TY = closer = less speed needed.
    // Lower TY = further = more speed needed.
    // TODO: replace FLYWHEEL_SPEED_POINTS with your actual tested values.
    private static final double[] TY_POINTS             = { -20.0, -15.0, -10.0, -5.0,  0.0,  5.0,  10.0 };
    private static final double[] FLYWHEEL_SPEED_POINTS = {  1.00,  1.00,  1.00, 1.00, 1.00, 1.00,  1.00 };
 
    // ── Logging fields ────────────────────────────────────────────────────────
    private double lastFlywheelOutput = 0.0;
 
    public ShooterSubsystem() {
        shooterRightC = new TalonFX(26);
        shooterLeftCC = new TalonFX(21);
        intakeMotor   = new TalonFX(20);
 
        topRequest    = new DutyCycleOut(0);
        bottomRequest = new DutyCycleOut(0);
        configA       = new TalonFXConfiguration();
        configB       = new TalonFXConfiguration();
 
        configA.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        configB.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterRightC.getConfigurator().apply(configA);
        shooterLeftCC.getConfigurator().apply(configB);
    }
 
    // ── Commands ──────────────────────────────────────────────────────────────
 
    public Command intake(double speed) {
        return runOnce(() -> intakeMotor.set(speed));
    }
 
    public Command intakestop() {
        return runOnce(() -> intakeMotor.set(0));
    }
 
    public Command stop() {
        return runOnce(() -> {
            shooterRightC.setControl(topRequest.withOutput(0));
            shooterLeftCC.setControl(bottomRequest.withOutput(0));
            intakeMotor.set(0);
        });
    }
 
    // ── TY-based flywheel speed ───────────────────────────────────────────────
 
    /**
     * Set flywheel speed based on Limelight TY value using lookup table.
     * Interpolates smoothly between tested data points.
     * Call every loop while shoot button is held.
     */
    public void setFlywheelFromTY(double ty) {
        if (ty <= TY_POINTS[0]) {
            setFlywheel(FLYWHEEL_SPEED_POINTS[0]); return;
        }
        if (ty >= TY_POINTS[TY_POINTS.length - 1]) {
            setFlywheel(FLYWHEEL_SPEED_POINTS[FLYWHEEL_SPEED_POINTS.length - 1]); return;
        }
        for (int i = 0; i < TY_POINTS.length - 1; i++) {
            if (ty >= TY_POINTS[i] && ty <= TY_POINTS[i + 1]) {
                double t = (ty - TY_POINTS[i]) / (TY_POINTS[i + 1] - TY_POINTS[i]);
                double output = FLYWHEEL_SPEED_POINTS[i] + t * (FLYWHEEL_SPEED_POINTS[i + 1] - FLYWHEEL_SPEED_POINTS[i]);
                setFlywheel(output);
                return;
            }
        }
    }
 
    // ── Helpers ───────────────────────────────────────────────────────────────
 public Command setFlywheel(double output) {
        return runOnce(() -> {
                shooterRightC.setControl(topRequest.withOutput(output));
                shooterLeftCC.setControl(bottomRequest.withOutput(output));
            });
            }
         
            // ── Periodic ──────────────────────────────────────────────────────────────
         
            
        
          
        
            @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/TY", LimelightHelpers.getTY("limelight"));
        SmartDashboard.putNumber("Shooter/FlywheelOutput", lastFlywheelOutput);
    }
}
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import java.time.temporal.ValueRange;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterRightC;
    private final TalonFX shooterLeftCC;
    private final TalonFX intakeMotor;
    private final DutyCycleOut topRequest;
     private final DutyCycleOut bottomRequest;
     private final TalonFXConfiguration configA;
     private final TalonFXConfiguration configB;


    

    public ShooterSubsystem() {
        // Motors spin opposite directions to both push the ball the same way
        shooterRightC = new TalonFX(26);
        shooterLeftCC = new TalonFX(21);
        intakeMotor = new TalonFX(20);

    topRequest = new DutyCycleOut(0);
     bottomRequest = new DutyCycleOut(0);
        configA = new TalonFXConfiguration();
        configB = new TalonFXConfiguration();
    configA.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configB.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterRightC.getConfigurator().apply(configA);
    shooterLeftCC.getConfigurator().apply(configB);
    }

    public Command shoot(double voltage) {
        return runOnce(() -> shooterRightC.setControl(topRequest.withOutput(voltage)))
        .andThen(
        runOnce(() ->shooterLeftCC.setControl(bottomRequest.withOutput(voltage))));
    }
    
    public Command intake(double speed) {
        return runOnce(() -> intakeMotor.set(speed));
    }

    public Command intakestop() {
        return runOnce(() -> intakeMotor.set(0));
    }

    public Command stop() {
        return runOnce(() -> shooterRightC.setControl(topRequest.withOutput(0)))
        .andThen(
        runOnce(() ->shooterLeftCC.setControl(bottomRequest.withOutput(0))));
    }
}
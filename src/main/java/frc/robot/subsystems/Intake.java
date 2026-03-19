// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final TalonFX intakee;
  private final TalonFX intakeMover;
  

  // ── Arm positions (in motor rotations) ────────────────────────────────────
  // TODO: run the arm to each position and read "IntakeArm/CurrentPosition"
  // from SmartDashboard to find the correct values.


  // ── PID ───────────────────────────────────────────────────────────────────
  private static final double kP = 60; // TODO: tune
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  /** Creates a new Intake. */
  public Intake() {
    intakee = new TalonFX(31);
    intakeMover = new TalonFX(13);
    

    TalonFXConfiguration moverConfig = new TalonFXConfiguration();
    moverConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    moverConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: flip if needed
    moverConfig.Slot0.kP = kP;
    moverConfig.Slot0.kI = kI;
    moverConfig.Slot0.kD = kD;
    moverConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
    moverConfig.MotorOutput.PeakReverseDutyCycle = -1.0;
    intakeMover.getConfigurator().apply(moverConfig);

    // Zero the encoder at startup.
    // Make sure the arm is fully retracted (IN) before enabling the robot.
    intakeMover.setPosition(0);
  }

  // ── Intake roller commands (existing) ─────────────────────────────────────

  public Command runIntake(double speed) {
    return runOnce(() -> intakee.set(speed));
  }

  public Command stopIntake() {
    return runOnce(() -> intakee.set(0));
  }

  // ── Arm position commands (new) ───────────────────────────────────────────



 
  

  /** Stop the arm motor. */
  public Command stopArm() {
    return runOnce(() -> intakeMover.stopMotor());
  }

    public Command goin(double speed) { //added to just make it move out to test
    return runOnce(() -> intakeMover.set(speed));
}

  public Command goOut(double speed) { //added to just make it move out to test
    return runOnce(() -> intakeMover.set(speed));
}

  // ─────────────────────────────────────────────────────────────────────────
  // Periodic
  // ─────────────────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeArm/CurrentPosition", intakeMover.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("IntakeArm/AppliedOutput", intakeMover.getMotorVoltage().getValueAsDouble());
  }
}
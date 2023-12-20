// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIDCoefficients;
import frc.robot.Constants.RobotConstants;
import com.revrobotics.SparkMaxRelativeEncoder;

public class Chassis extends SubsystemBase {
  public final DifferentialDrive m_drive;

  private final SparkMaxPIDController m_left_pid, m_right_pid;
  private final RelativeEncoder m_left_encoder, m_right_encoder;

  /** Creates a new Chassis. */
  public Chassis() {
    /*
     * Initialize the motors
     */

    CANSparkMax m_left_lead = new CANSparkMax(RobotConstants.kLeftFrontMotorPort, MotorType.kBrushless);
    CANSparkMax m_left_follow = new CANSparkMax(RobotConstants.kLeftBackMotorPort, MotorType.kBrushless);
    CANSparkMax m_right_lead = new CANSparkMax(RobotConstants.kRightBackMotorPort, MotorType.kBrushless);
    CANSparkMax m_right_follow = new CANSparkMax(RobotConstants.kRightBackMotorPort, MotorType.kBrushless);

    /*
     * MotoSetting the Motors
     */

    m_left_follow.follow(m_left_lead);
    m_right_follow.follow(m_right_lead);

    m_left_follow.close();

    m_right_follow.close();
    /*
     * PID
     */

    this.m_right_pid = m_right_lead.getPIDController();
    this.m_left_pid = m_left_lead.getPIDController();

    this.m_right_pid.setP(PIDCoefficients.kP);
    this.m_left_pid.setP(PIDCoefficients.kP);

    this.m_right_pid.setI(PIDCoefficients.kI);
    this.m_left_pid.setI(PIDCoefficients.kI);

    this.m_right_pid.setD(PIDCoefficients.kD);
    this.m_left_pid.setD(PIDCoefficients.kD);

    this.m_right_pid.setIZone(PIDCoefficients.kIz);
    this.m_left_pid.setIZone(PIDCoefficients.kIz);

    this.m_right_pid.setFF(PIDCoefficients.kFF);
    this.m_left_pid.setFF(PIDCoefficients.kFF);

    this.m_right_pid.setOutputRange(PIDCoefficients.kMinOutput, PIDCoefficients.kMaxOutput);
    this.m_left_pid.setOutputRange(PIDCoefficients.kMinOutput, PIDCoefficients.kMaxOutput);

    /*
     * Put values to Smartdashboard
     */
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", PIDCoefficients.kP);
    SmartDashboard.putNumber("I Gain", PIDCoefficients.kI);
    SmartDashboard.putNumber("D Gain", PIDCoefficients.kD);
    SmartDashboard.putNumber("I Zone", PIDCoefficients.kIz);
    SmartDashboard.putNumber("Feed Forward", PIDCoefficients.kFF);
    SmartDashboard.putNumber("Max Output", PIDCoefficients.kMaxOutput);
    SmartDashboard.putNumber("Min Output", PIDCoefficients.kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

    /*
     * Encoder
     */

    this.m_left_encoder = m_right_lead.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 4096);
    this.m_right_encoder = m_left_lead.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 4096);

    /*
     * Encoder and PID
     */

      this.m_right_pid.setFeedbackDevice(m_right_encoder);
      this.m_left_pid.setFeedbackDevice(m_left_encoder);

    /*
     * Differential Drive
     */
    this.m_drive = new DifferentialDrive(m_left_lead, m_right_lead);
    this.m_drive.setSafetyEnabled(false);
  }

  @Override
  public void periodic() {

    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != PIDCoefficients.kP)) {
      this.m_right_pid.setP(p);
      this.m_left_pid.setP(p);
      PIDCoefficients.kP = p;
    }
    if ((i != PIDCoefficients.kI)) {
      this.m_right_pid.setI(i);
      this.m_left_pid.setI(i);
      PIDCoefficients.kI = i;
    }
    if ((d != PIDCoefficients.kD)) {
      this.m_right_pid.setD(d);
      this.m_left_pid.setD(d);
      PIDCoefficients.kD = d;
    }
    if ((iz != PIDCoefficients.kIz)) {
      this.m_right_pid.setIZone(iz);
      this.m_left_pid.setIZone(iz);
      PIDCoefficients.kIz = iz;
    }
    if ((ff != PIDCoefficients.kFF)) {
      this.m_right_pid.setFF(ff);
      this.m_left_pid.setFF(ff);
      PIDCoefficients.kFF = ff;
    }
    if ((max != PIDCoefficients.kMaxOutput) || (min != PIDCoefficients.kMinOutput)) {
      this.m_right_pid.setOutputRange(min, max);
      this.m_left_pid.setOutputRange(min, max);
      PIDCoefficients.kMinOutput = min;
      PIDCoefficients.kMaxOutput = max;
    }

    this.m_right_pid.setReference(rotations, CANSparkMax.ControlType.kPosition);
    this.m_left_pid.setReference(rotations, CANSparkMax.ControlType.kPosition);

    // start the drive
    this.m_drive.feed();
  }
}

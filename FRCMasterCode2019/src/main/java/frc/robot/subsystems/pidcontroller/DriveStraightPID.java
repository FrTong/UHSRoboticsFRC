/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.pidcontroller;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.Constant;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveStraightPID extends PIDSubsystem {
  /**
   * Add your docs here.
   */
  private double power;

  public DriveStraightPID() {
    // Insert a subsystem name and PID values here
    super("DriveStraightPID", 1, 0, 0);
    setAbsoluteTolerance(2.0); //Degree of error
    getPIDController().setContinuous(true);
    setInputRange(-360, 360);
    setOutputRange(-Constant.PID_DRIVE_OUTPUT, Constant.PID_DRIVE_OUTPUT);
    setSetpoint(0);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }

  public void setPower(double power){
    this.power = power;
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    return RobotMap.gyro.getAngle();
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    Robot.driveSubsystem.drive(power+output, power+output);
  }
  
  @Override
  public void disable(){
    super.disable();
    Robot.driveSubsystem.stopMotor();
  }
}

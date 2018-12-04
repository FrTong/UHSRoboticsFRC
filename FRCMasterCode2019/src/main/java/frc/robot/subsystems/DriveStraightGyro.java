/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveStraightGyro extends PIDSubsystem {
  /**
   * Add your docs here.
   */
  double pidOutputRange = 0.5;
  double drivePow;
  double startTime;
  public DriveStraightGyro(double drivePow,double second) {
    // Intert a subsystem name and PID values here
    super("DriveStraightGyro", 0.1, 0, 0);
    setAbsoluteTolerance(0.5);
		getPIDController().setContinuous(false);
		setInputRange(-360,360);  //angle degree
		setOutputRange(-pidOutputRange,pidOutputRange);
		this.drivePow = drivePow;
    RobotMap.gyro.reset();
    setSetpoint(second);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return RobotMap.gyro.getAngle();
  }

  @Override
  protected void usePIDOutput(double output) {
    RobotMap.left1.pidWrite(output);
    RobotMap.left2.pidWrite(output);
    RobotMap.right1.pidWrite(output);
    RobotMap.right2.pidWrite(output);
  }
}

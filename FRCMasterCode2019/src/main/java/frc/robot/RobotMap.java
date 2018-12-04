/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  //DRIVE
  public static Spark left1 = new Spark(0);
  public static Spark left2 = new Spark(1);
  public static Spark right1 = new Spark(2);
  public static Spark right2 = new Spark(3);
  public static Encoder leftEncoder = new Encoder(0,1,false,EncodingType.k4X);
  public static Encoder rightEncoder = new Encoder(2,3,false,EncodingType.k4X);
  public static AnalogGyro gyro = new AnalogGyro(4);
}

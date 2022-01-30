// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /*--------------/
 /----Buttons----/
/--------------*/

//Shooter
public static int shootButton;
public static int spinUpButton = 0; //Angle of the POV
public static int idleButton = 180; //Angle of the POV

//Hood
public static int adjustAxis;
public static int autoAdjustBotton;

//Intake
public static int lowerButton;
public static int raiseButton;

  /*--------------/
 /----Shooter----/
/--------------*/

public static float shooterIdleSpeed = 0.5f; //Idling Speed
public static float spinUpSpeed = 0.85f; //Speed of launch
public static float loadSpeed = 0.5f; //Speed of ball loading




  /*---------------/
 /------Hood------/
/---------------*/

//NOT FINAL VALUES
//Calibration Values
public static double hoodAngleMax = 60;
public static double hoodAngleMin = 30;

//Soft caps
public static double softAngleMax = 55; 
public static double softAngleMin = 35;

public static double angleAdjustSpeed = 1;
public static double angleTolerance = 1; //Tolerance for angle adjust


  /*-------------/
 /----Intake----/
/-------------*/

public static float intakeSpeed = 0.85f;
public static float intakeIdleSpeed = -0.2f;








}

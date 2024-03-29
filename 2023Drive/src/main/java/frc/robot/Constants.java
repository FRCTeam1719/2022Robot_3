// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

/**
 * The Constants class provides a 
 * convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

// controlers    
    public static final int DRIVE_XBOX_CONTROLLER = 0;
    public static final int HELPER_XBOX_CONTROLLER = 1;
// left drive
    public static final int LEFT_MOTOR_CAN1_ID = 1; // change to 1
    public static final int LEFT_MOTOR_CAN2_ID = 2; // change to 2
//right drive
    public static final int RIGHT_MOTOR_CAN1_ID = 3; // change to 3
    public static final int RIGHT_MOTOR_CAN2_ID = 4; // change to 4
// arm 
    public static final int ARM_ROTATE_CAN_ID = 5; // change to 5
    public static final int ARM_EXTEND_CAN_ID = 6; // change to 6
// Solenoids
    public static final int GRAB_ID = 0; // change to 0
    public static final int BREAK_ID = 2; // change to 2  
    
// Setpoints
public static final double INCHES_FROM_GOAL = 24; // change to 4

// Placeholders
public static final double Zero_for_now = 0; // change to 0

// Speed Regulator
public static final double SPEED_REGULATOR = 0.65;
public static final double DEAD_ZONE_VALUE = 0.03;
public static final double TURN_REGULATOR = 0.6;
public static final double ARMSPEED_SCALER = 0.65;
public static final double ARMEXTEND_SCALER = 0.75;

// Time of Flight
public static final int TIMEOFFLIGHT_ID = 0;

public static final double MAX_DISTANCE = 1000;


public static final double ARM_BEGIN = 300;
public static final double ARM_MID = 500;
public static final double ARM_LONG = 1000;

// Movement PID's 


public static final double DRIVEkp = .5;
public static final double DRIVEki = 0;
public static final double DRIVEkd = 0;

// PercentPositions for encoder extension
 
public static final double PERCENT_BEGIN = 0; // Change to 0
public static final double PERCENT_MID = 50;
public static final double PERCENT_LONG = 90;
public static final double PERCENT_MAX = 100; //Change to 100
public static final double PERCENT_MAX_HEIGHT = 80; //Change to 100

// armangles

public static final double MAX_ROTATE = 0.285; // change to 0.275



// GearRatios
public static final double ROTATE_GEAR_RATIO = 130;
public static final double EXTEND_MAX_CXYCLES = 100;

}
 
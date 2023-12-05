// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static double speedScale = 1.0;

  public static final class Auton
  {

    public static final PIDFConfig xAutoPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig yAutoPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_SPEED        = 4;
    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;
  }

      /* MOTOR and ENCODER IDS */
      public static final int INTAKE_ROLLER_MOTOR_1_ID = 21;
      public static final int MAIN_ARM_MOTOR_1_ID = 17;
      public static final int MAIN_ARM_MOTOR_2_ID = 18;
      public static final int INTAKE_ARM_MOTOR_ID = 19;
      public static final int INTAKE_ARM_ENCODER_PWM_CHANNEL = 0;
      public static final int MAIN_ARM_ENCODER_PWM_CHANNEL = 9;
      public static final int PIGEON_DEVICE_ID = 16;
      public static final double ARM_COMMAND_CHECK_LIMIT = 500;
      /* Robot Arm Position Combos */
      public static final class Start_Arm_Position {   // starting position
          public static final double intake_arm_position = -1026;
          public static final double main_arm_position = -1400;
      }
      public static final class Cube_Ground_Pickup_Position {   // cube pickup position
          public static final double intake_arm_position = -20350.0;//-22040.0;//-22575.0;//-22538;
          public static final double main_arm_position = -4770.0;//-5698.0;//-7450.0;
      }
      public static final class Cube_Station_Pickup_Position { 
            // cube pickup position
          public static final double intake_arm_position = 0.0;
          public static final double main_arm_position = 0.0;
      }
      public static final class Cone_Ground_Upright_Pickup_Position {   // cone pickup position
          public static final double intake_arm_position = -39515;
          public static final double main_arm_position = -21997;
      }
      public static final class Cone_Ground_Side_Pickup_Position {   // cube pickup position
          public static final double intake_arm_position = -31074;//-26529.0;//-33351.0;//-33524;
          public static final double main_arm_position = -10256;//-7033.0;//-10097.0;//-10258;
      }
      public static final class Cone_Station_Pickup_Position {   // cube pickup position
          public static final double intake_arm_position = -10098;
          public static final double main_arm_position = -5112;
      }
      public static final class Cone_Community_Score_Position {   // cone middle scoring position
          public static final double intake_arm_position = -4530;
          public static final double main_arm_position = -77000;
      }
      public static final class Cone_Mid_Score_Position {   // cone middle scoring position
          public static final double intake_arm_position = -17272;//-10642;
          public static final double main_arm_position = -77358;//-73481;
      }
      public static final class Cone_Top_Score_Position {   // cone top scoring position
          public static final double intake_arm_position = -21000; //substract to make higher
          public static final double main_arm_position = -73027; //add to make it rotate less
      }
      public static final class Cube_Community_Score_Position {   // cube middle scoring position
          public static final double intake_arm_position = -1026;
          public static final double main_arm_position = -1400;
      }
      public static final class Cube_Mid_Score_Position {   // cube middle scoring position
          public static final double intake_arm_position = -5900;
          public static final double main_arm_position = -83358;
      }
      public static final class Cube_Top_Score_Position {   // cube top scoring position
          public static final double intake_arm_position = -27000.0;
          public static final double main_arm_position = -84183.0;
      }
}

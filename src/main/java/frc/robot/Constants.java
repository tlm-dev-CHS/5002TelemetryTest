// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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

 

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;

    //elevator Motor controllers
    public final static int m_elevator = 2;
    public final static int m_elevatorFollower = 6;
    
    public static final double elvatorConversionFactor = 1;

    //Arm Motor controllers
    public final int m_armRotator = 15;
    public final int m_armShooter = 1;

    //Climber Motor Controllers
    public final int m_climber = 4;

    //Arm Constants
    public final int beamBreakId = 0;
  }

  public static final class RobotConstants{

    public static final double elevatorHeight = 28.0;
    

  }
}
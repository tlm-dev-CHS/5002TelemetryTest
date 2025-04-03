// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Thread.State;
import java.util.Arrays;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.autoAlign;

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
    
    public final static double elvatorConversionFactor = 0.59635;

    //Arm Motor controllers
    public final static int m_armRotator = 15;
    public final static int m_armShooter = 1;

    public final static double m_armConversionFactor = 1.5192950471;

    //Climber Motor Controllers
    public final static int m_climber = 4;

    //AutoAIm:
    public static final String cameraName = "photonvision";
    public static final double distanceToTag = 16 * 2.54;


    //Arm Constants
    public final int beamBreakId = 0;

    //Vision
  }

  public static final class RobotConstants{

    public static final double elevatorHeight = 28.0;
  }

  public enum AutoAlignStates {

    BLUE_POS_1_LEFT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),
    BLUE_POS_1_RIGHT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),

    BLUE_POS_2_LEFT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),
    BLUE_POS_2_RIGHT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),

    BLUE_POS_3_LEFT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),
    BLUE_POS_3_RIGHT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),

    BLUE_POS_4_LEFT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),
    BLUE_POS_4_RIGHT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),

    BLUE_POS_5_LEFT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),
    BLUE_POS_5_RIGHT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),

    BLUE_POS_6_LEFT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),
    BLUE_POS_6_RIGHT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),

    RED_POS_1_LEFT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),
    RED_POS_1_RIGHT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),

    RED_POS_2_LEFT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),
    RED_POS_2_RIGHT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),

    RED_POS_3_LEFT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),
    RED_POS_3_RIGHT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),

    RED_POS_4_LEFT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),
    RED_POS_4_RIGHT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),

    RED_POS_5_LEFT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),
    RED_POS_5_RIGHT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),

    RED_POS_6_LEFT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168))),
    RED_POS_6_RIGHT(new Pose2d(new Translation2d(14.42, 4.5), Rotation2d.fromDegrees(168)));

    private final Pose2d pose;

    AutoAlignStates(Pose2d pose) {
        this.pose = pose;
    }

    public Pose2d getPose() {
        return pose;
    }
  }

  public static final Map<String, Map> autoAlignSide;

  public static final Map<String, Map> blueAutoPoses;
  public static final Map<String, Map> redAutoPoses;

  public static final Map<Integer, Pose2d> blueAutoPosesRight;
  public static final Map<Integer, Pose2d> blueAutoPosesLeft;

  public static final Map<Integer, Pose2d> redAutoPosesRight;
  public static final Map<Integer, Pose2d> redAutoPosesLeft;


  static {
    autoAlignSide = new HashMap<String, Map>();

    blueAutoPoses = new HashMap<String, Map>();
    redAutoPoses = new HashMap<String, Map>();

    blueAutoPosesRight = new HashMap<Integer, Pose2d>();
    blueAutoPosesLeft = new HashMap<Integer, Pose2d>();

    redAutoPosesRight = new HashMap<Integer, Pose2d>();
    redAutoPosesLeft = new HashMap<Integer, Pose2d>();

    blueAutoPosesRight.put(21, AutoAlignStates.BLUE_POS_1_RIGHT.getPose());
    blueAutoPosesLeft.put(21,AutoAlignStates.BLUE_POS_1_LEFT.getPose());

    blueAutoPosesRight.put(22, AutoAlignStates.BLUE_POS_2_RIGHT.getPose());
    blueAutoPosesLeft.put(22,AutoAlignStates.BLUE_POS_2_LEFT.getPose());

    blueAutoPosesRight.put(17, AutoAlignStates.BLUE_POS_3_RIGHT.getPose());
    blueAutoPosesLeft.put(17,AutoAlignStates.BLUE_POS_3_LEFT.getPose());

    blueAutoPosesRight.put(18, AutoAlignStates.BLUE_POS_4_RIGHT.getPose());
    blueAutoPosesLeft.put(18,AutoAlignStates.BLUE_POS_4_LEFT.getPose());

    blueAutoPosesRight.put(19, AutoAlignStates.BLUE_POS_5_RIGHT.getPose());
    blueAutoPosesLeft.put(19,AutoAlignStates.BLUE_POS_5_LEFT.getPose());

    blueAutoPosesRight.put(20, AutoAlignStates.BLUE_POS_6_RIGHT.getPose());
    blueAutoPosesLeft.put(20,AutoAlignStates.BLUE_POS_6_LEFT.getPose());

    redAutoPosesRight.put(10, AutoAlignStates.RED_POS_1_RIGHT.getPose());
    redAutoPosesLeft.put(10,AutoAlignStates.BLUE_POS_1_LEFT.getPose());

    redAutoPosesRight.put(9, AutoAlignStates.RED_POS_2_RIGHT.getPose());
    redAutoPosesLeft.put(9,AutoAlignStates.BLUE_POS_2_LEFT.getPose());

    redAutoPosesRight.put(8, AutoAlignStates.RED_POS_3_RIGHT.getPose());
    redAutoPosesLeft.put(8,AutoAlignStates.BLUE_POS_3_LEFT.getPose());

    redAutoPosesRight.put(7, AutoAlignStates.RED_POS_4_RIGHT.getPose());
    redAutoPosesLeft.put(7,AutoAlignStates.BLUE_POS_4_LEFT.getPose());

    redAutoPosesRight.put(6, AutoAlignStates.RED_POS_5_RIGHT.getPose());
    redAutoPosesLeft.put(6,AutoAlignStates.BLUE_POS_5_LEFT.getPose());

    redAutoPosesRight.put(11, AutoAlignStates.RED_POS_6_RIGHT.getPose());
    redAutoPosesLeft.put(11,AutoAlignStates.BLUE_POS_6_LEFT.getPose());

    blueAutoPoses.put("Right", blueAutoPosesRight);
    blueAutoPoses.put("Left", blueAutoPosesLeft);

    redAutoPoses.put("Right", redAutoPosesRight);
    redAutoPoses.put("Left", redAutoPosesLeft);

    autoAlignSide.put("Blue", blueAutoPoses);
    autoAlignSide.put("Red", redAutoPoses);
    
    
  }
}

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.CustomTypes.PID_Values;
import frc.robot.CustomTypes.SwerveCanIDs;
import frc.robot.CustomTypes.SwerveModuleLocations;
import frc.robot.CustomTypes.Math.Vector2;

  public final class Constants {
    public static class OperatorConstants {
      public static final int kDriverControllerPort = 0;
    }

    public static final class ControllerConstants {
      public static final int DRIVER_STICK_CHANNEL = 0;
      public static final int AUX_STICK_CHANNEL = 1;
      public static final int XBOXCONTROLLER_CHANNEL = 3;

      public static final double DEADZONE_DRIVE = 0.1;
      public static final double DEADZONE_STEER = 0.3;
    }

    public static final class DriveConstants {
      // Drivetrain Motor and Encoder IDs

      // #region <Robot 2024 Constants>
      public static final SwerveCanIDs Robot2024Can = new SwerveCanIDs(
          2, // LEFT_FRONT_DRIVE_MOTOR_ID 
          1, // LEFT_FRONT_STEER_MOTOR_ID 
          3, // RIGHT_FRONT_DRIVE_MOTOR_ID 
          4, // RIGHT_FRONT_STEER_MOTOR_ID 
          5, // LEFT_REAR_DRIVE_MOTOR_ID 
          6, // LEFT_REAR_STEER_MOTOR_ID 
          8, // RIGHT_REAR_DRIVE_MOTOR_ID 
          7, // RIGHT_REAR_STEER_MOTOR_ID 
          10, // LEFT_FRONT_STEER_ENCODER_ID 
          11, // RIGHT_FRONT_STEER_ENCODER_ID 
          12, // LEFT_REAR_STEER_ENCODER_ID 
          13); // RIGHT_REAR_STEER_ENCODER_ID 

      public static final SwerveModuleLocations Robot2024SwerveLocations = new SwerveModuleLocations(
          ((29 / 2) - 3.25) * MathConstants.INCH_TO_METER, // LEFT_FRONT_WHEEL_X
          ((30 / 2) - 3.25) * MathConstants.INCH_TO_METER, // LEFT_FRONT_WHEEL_Y
          ((29 / 2) - 3.25) * MathConstants.INCH_TO_METER, // RIGHT_FRONT_WHEEL_X
          (-(30 / 2) + 3.25) * MathConstants.INCH_TO_METER, // RIGHT_FRONT_WHEEL_Y
          (-(29 / 2) + 3.25) * MathConstants.INCH_TO_METER, // RIGHT_REAR_WHEEL_X
          (-(30 / 2) + 3.25) * MathConstants.INCH_TO_METER, // RIGHT_REAR_WHEEL_Y
          (-(29 / 2) + 3.25) * MathConstants.INCH_TO_METER, // LEFT_REAR_WHEEL_X
          ((30 / 2) - 3.25) * MathConstants.INCH_TO_METER); //LEFT_REAR_WHEEL_Y
      // #endregion

      // #region <Robot 2023 Constants>
        public static final SwerveCanIDs Robot2023SwerveCAN = new SwerveCanIDs(
          1, // LEFT_FRONT_DRIVE_MOTOR_ID 
          2, // LEFT_FRONT_STEER_MOTOR_ID 
          3, // RIGHT_FRONT_DRIVE_MOTOR_ID 
          4, // RIGHT_FRONT_STEER_MOTOR_ID 
          5, // LEFT_REAR_DRIVE_MOTOR_ID 
          6, // LEFT_REAR_STEER_MOTOR_ID 
          7, // RIGHT_REAR_DRIVE_MOTOR_ID 
          8, // RIGHT_REAR_STEER_MOTOR_ID 
          10, // LEFT_FRONT_STEER_ENCODER_ID 
          11, // RIGHT_FRONT_STEER_ENCODER_ID 
          12, // LEFT_REAR_STEER_ENCODER_ID 
          13); // RIGHT_REAR_STEER_ENCODER_ID 

        public static final SwerveModuleLocations Robot2023SwerveLocations = new SwerveModuleLocations(
          (11 - 2.25) * MathConstants.INCH_TO_METER, // LEFT_FRONT_WHEEL_X
          (14 - 2.25) * MathConstants.INCH_TO_METER, // LEFT_FRONT_WHEEL_Y
          (11 - 2.25) * MathConstants.INCH_TO_METER, // RIGHT_FRONT_WHEEL_X
          (-14 + 2.25) * MathConstants.INCH_TO_METER, // RIGHT_FRONT_WHEEL_Y
          (-11 + 2.25) * MathConstants.INCH_TO_METER, // RIGHT_REAR_WHEEL_X
          (-14 + 2.25) * MathConstants.INCH_TO_METER, // RIGHT_REAR_WHEEL_Y
          (-11 + 2.25) * MathConstants.INCH_TO_METER, // LEFT_REAR_WHEEL_X
          (14 - 2.25) * MathConstants.INCH_TO_METER); //LEFT_REAR_WHEEL_Y
      // #endregion

      // #region <CaidBot Constants>
          public static final SwerveCanIDs CaidBotSwerveCAN = new SwerveCanIDs(
            1, // LEFT_FRONT_DRIVE_MOTOR_ID 
            2, // LEFT_FRONT_STEER_MOTOR_ID 
            5, // RIGHT_FRONT_DRIVE_MOTOR_ID 
            6, // RIGHT_FRONT_STEER_MOTOR_ID 
            3, // LEFT_REAR_DRIVE_MOTOR_ID 
            4, // LEFT_REAR_STEER_MOTOR_ID 
            7, // RIGHT_REAR_DRIVE_MOTOR_ID
            8, // RIGHT_REAR_STEER_MOTOR_ID
            10, // LEFT_FRONT_STEER_ENCODER_ID
            12, // RIGHT_FRONT_STEER_ENCODER_ID 
            11, // LEFT_REAR_STEER_ENCODER_ID
           13); // // RIGHT_REAR_STEER_ENCODER_ID

        public static final SwerveModuleLocations CaidBotSwerveLocations = new SwerveModuleLocations(
          11.75 * MathConstants.INCH_TO_METER, // LEFT_FRONT_WHEEL_X
          11.75 * MathConstants.INCH_TO_METER, // LEFT_FRONT_WHEEL_Y
          11.75 * MathConstants.INCH_TO_METER, // RIGHT_FRONT_WHEEL_X
          -11.75 * MathConstants.INCH_TO_METER, // RIGHT_FRONT_WHEEL_Y
          -11.75 * MathConstants.INCH_TO_METER, // RIGHT_REAR_WHEEL_X
          -11.75 * MathConstants.INCH_TO_METER, // RIGHT_REAR_WHEEL_Y
          -11.75 * MathConstants.INCH_TO_METER, // LEFT_REAR_WHEEL_X
          11.75 * MathConstants.INCH_TO_METER); //LEFT_REAR_WHEEL_Y
      // #endregion

      // CHANGE TO SET CURRENT ROBOT INFO //
      public static final SwerveCanIDs ROBOT_SWERVE_CAN = Robot2024Can;
      public static final SwerveModuleLocations ROBOT_SWERVE_LOCATIONS = Robot2024SwerveLocations;
      //////////////////////////////////////

      public static final double WHEEL_RADIUS = 2.0 * 0.0254; // meters * 0.98
      public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS; // meters/revolution

      public static final double MAX_DRIVE_SPEED = 3.5; // meters/second
      public static final double MAX_STEER_RATE = .5; // rotations/second of a wheel for steer.
      public static final double MAX_TWIST_RATE = .6 * 2.0 * Math.PI; // radians/second of the robot rotation.
      public static final double CONTROLLER_TWIST_RATE = 2; // constant turn rate for using controller

      // #region <Misc CAN IDs>
        public static final int PIGEON_IMU_ID = 9;
        public static final int PIGEON_2_ID = 9;
      // #endregion

      // #region <Gear Ratios>
      // Drive motor gear ratio.
      // | Driving Gear | Driven Gear |
      // First Stage | 14 | 50 |
      // Second Stage | 28 | 16 |
      // Third Stage | 15 | 60 |
      //
      // Overall Gear Ratio = .169
      // One rotation of the motor gives .169 rotations of the wheel.
      // 5.9 rotations of the motor gives one rotation of the wheel.
      // https://www.swervedrivespecialties.com/products/mk4n-swerve-module
      public static final double DRIVE_GEAR_RATIO = .169f;

      // Steer motor gear ratio
      // | Driving Gear | Driven Gear |
      // First Stage | 15 | 32 |
      // Second Stage | 10 | 40 |
      //
      // Overall Gear Ration = 0.1171875
      // One rotation of the motor gives 0.1171875 rotations of the wheel.
      // 8.533 rotations of the motor gives one rotation of the wheel.
      public static final double STEER_GEAR_RATIO = (15.0 / 32) * (10 / 40);
      // #endregion

      // #region <PID Values>
      public static final PID_Values PID_SparkMax_Steer = new PID_Values(0.0001, 0, 0, 0, 0.00005);
      public static final PID_Values PID_Encoder_Steer = new PID_Values(20, 10, 0);
      public static final PID_Values PID_SparkMax_Drive = new PID_Values(0.0003, 0, 0, 0, 0.00016);
      // #endregion

      public static final double AUTO_ODOMETRY_DRIVE_MIN_SPEED = .1;
      public static final double AUTO_ODOMETRY_DRIVE_MAX_SPEED = 2;
      public static final double AUTO_ODOMETRY_DRIVE_TARGET_ALLOWED_ERROR = .1; // in meters
      public static final double AUTO_ODOMETRY_DRIVE_SLOWDOWN_DISTANCE = .6; // in meters
  }

  public static final class MathConstants
  {
    public static final double INCH_TO_METER = 0.0254;
  }
}
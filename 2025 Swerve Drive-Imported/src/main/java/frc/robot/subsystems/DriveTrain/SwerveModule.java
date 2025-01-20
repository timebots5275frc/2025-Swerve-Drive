package frc.robot.subsystems.DriveTrain;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants.DriveConstants;

public class SwerveModule {

    public SparkFlex driveMotor;
    private SparkMax steerMotor;
    public RelativeEncoder driveNEOVortexMotorEncoder; // NEO build-in Encoder

    private CANcoder steerAngleEncoder;

    private PIDController steerAnglePID;
    private SparkClosedLoopController steerMotorVelocityPID;
    private SparkClosedLoopController driveMotorVelocityPID;

    public SwerveModule(int driveMotorID, int steerMotorID, int steerEncoderId) {

        driveMotor = new SparkFlex(driveMotorID, MotorType.kBrushless);
        steerMotor = new SparkMax(steerMotorID, MotorType.kBrushless);

        steerAngleEncoder = new CANcoder(steerEncoderId);

        driveNEOVortexMotorEncoder = driveMotor.getEncoder();
        //driveNEOVortexMotorEncoder.setPositionConversionFactor();

        /// PID Controllers ///
        steerAnglePID = new PIDController(DriveConstants.PID_Encoder_Steer.P, DriveConstants.PID_Encoder_Steer.I, DriveConstants.PID_Encoder_Steer.D);
        steerAnglePID.enableContinuousInput(-180, 180);

        // Get the motor controller PIDs
        steerMotorVelocityPID = steerMotor.getClosedLoopController();
        driveMotorVelocityPID = driveMotor.getClosedLoopController();
        // set PID coefficients
        ClosedLoopConfig steerClosedLoopConfig = new ClosedLoopConfig();
        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerClosedLoopConfig.pidf(DriveConstants.PID_SparkMax_Steer.P, DriveConstants.PID_SparkMax_Steer.I, DriveConstants.PID_SparkMax_Steer.D, DriveConstants.PID_SparkMax_Steer.kFF);
        steerClosedLoopConfig.iZone(DriveConstants.PID_SparkMax_Steer.IZ);
        steerClosedLoopConfig.outputRange(-1, 1);
        steerConfig.apply(steerClosedLoopConfig);
        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

        // set PID coefficients
        ClosedLoopConfig driveClosedLoopConfig = new ClosedLoopConfig();
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveClosedLoopConfig.pidf(DriveConstants.PID_SparkFlex_Drive.P, DriveConstants.PID_SparkFlex_Drive.I, DriveConstants.PID_SparkFlex_Drive.D, DriveConstants.PID_SparkFlex_Drive.kFF);
        driveClosedLoopConfig.iZone(DriveConstants.PID_SparkFlex_Drive.IZ);
        driveClosedLoopConfig.outputRange(-1, 1);
        driveConfig.apply(driveClosedLoopConfig);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       
    }

    /**
     * Returns the current state of the module.
     * 
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        double driveSpeed = speedFromDriveRpm(driveNEOVortexMotorEncoder.getVelocity());
        double steerAngleRadians = Math.toRadians(steerAngleEncoder.getAbsolutePosition().getValueAsDouble() * 360);

        return new SwerveModuleState(driveSpeed, new Rotation2d(steerAngleRadians));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean logValues, String name) {

        double steerAngleDegrees = steerAngleEncoder.getAbsolutePosition().getValueAsDouble() * 360;
        double curSteerAngleRadians = Math.toRadians(steerAngleDegrees);

        // Optimize the reference state to avoid spinning further than 90 degrees
        var state = SwerveModuleState.optimize(desiredState, new Rotation2d(curSteerAngleRadians));

        // The output of the steerAnglePID becomes the steer motor rpm reference.
        double steerMotorRpm = steerAnglePID.calculate(steerAngleDegrees,
                state.angle.getDegrees());
       
        steerMotorVelocityPID.setReference(-steerMotorRpm, SparkMax.ControlType.kVelocity);

        double driveMotorRpm = driveRpmFromSpeed(state.speedMetersPerSecond);

        if (logValues) {
            double driveSpeed = driveNEOVortexMotorEncoder.getVelocity();
            SmartDashboard.putNumber(name + " DriveSpeedMetersPerSecond", state.speedMetersPerSecond);
            SmartDashboard.putNumber(name + " DriveMotorRpmCommand", driveMotorRpm);
            SmartDashboard.putNumber(name + " DriveMotorSpeed", driveSpeed);
        }

        driveMotorVelocityPID.setReference(driveMotorRpm, SparkMax.ControlType.kVelocity);
    }

    /**
     * Returns the required motor rpm from the desired wheel speed in meters/second
     * 
     * @param speedMetersPerSecond
     * @return rpm of the motor
     */
    public double driveRpmFromSpeed(double speedMetersPerSecond) {
        var rpm = speedMetersPerSecond * 60.0 / DriveConstants.WHEEL_CIRCUMFERENCE / DriveConstants.DRIVE_GEAR_RATIO;
        return -rpm; // rotation flip from gear count
    }

    /**
     * Returns the wheel speed in meters/second calculated from the drive motor rpm.
     * 
     * @param rpm
     * @return wheelSpeed
     */
    public double speedFromDriveRpm(double rpm) {
        var speedMetersPerSecond = rpm * DriveConstants.DRIVE_GEAR_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE / 60.0;
        return -1 * speedMetersPerSecond; // Rotation reversed due to gears.
    }

    public SwerveModulePosition getPosition() {
        double distance = driveNEOVortexMotorEncoder.getPosition();
        return new SwerveModulePosition(distance, new Rotation2d(Math.toRadians(steerAngleEncoder.getAbsolutePosition().getValueAsDouble())));
    }
}
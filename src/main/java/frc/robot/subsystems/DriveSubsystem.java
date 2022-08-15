package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.libs.VelocityDutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class DriveSubsystem {

    WPI_VictorSPX left_front = new WPI_VictorSPX(Constants.DriveTrainConstants.leftFrontCANID);
    WPI_VictorSPX left_back = new WPI_VictorSPX(Constants.DriveTrainConstants.leftBackCANID);

    WPI_VictorSPX right_front = new WPI_VictorSPX(Constants.DriveTrainConstants.rightFrontCANID);
    WPI_VictorSPX right_back = new WPI_VictorSPX(Constants.DriveTrainConstants.rightBackCANID);

    CANSparkMax mid_neo = new CANSparkMax(Constants.DriveTrainConstants.midMotorCANID, MotorType.kBrushless);

    MotorControllerGroup leftMotors = new MotorControllerGroup(left_front, left_back);
    MotorControllerGroup rightMotors = new MotorControllerGroup(right_front, right_back);

    DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);

    VelocityDutyCycleEncoder driveLeftEncoder = new VelocityDutyCycleEncoder(0);
    VelocityDutyCycleEncoder driveRightEncoder = new VelocityDutyCycleEncoder(1);

    ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

    private final DifferentialDriveOdometry m_odometry;

    public DriveSubsystem() {

        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
        m_odometry.resetPosition(new Pose2d(), m_gyro.getRotation2d());

        rightMotors.setInverted(true);
        leftMotors.setInverted(false);

        resetEncoders();
        m_gyro.reset();
        m_gyro.calibrate();

    }

    public void resetEncoders() {
        driveLeftEncoder.reset();
        driveRightEncoder.reset();
    }

    public double getLeftEncoderPosition() {
        return driveLeftEncoder.getDistance();
    }

    public double getRightEncoderPosition() {
        return driveRightEncoder.getDistance();
    }

    public double getLeftEncoderVelocity() {
        return driveLeftEncoder.getRate();
    }

    public double getRightEncoderVelocity() {
        return driveRightEncoder.getRate();
    }

    public double getTurnRate() {
        return -m_gyro.getRate();
    }

    public double getHeading() {
        return m_gyro.getAngle();
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(rightVolts);
        m_drive.feed();
    }

    public double getAverageEncoderDistance() {
        return ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2);
    }

    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);

    }

    public void zeroHeading() {
        m_gyro.calibrate();
        m_gyro.reset();
    }

    public ADXRS450_Gyro getGyro() {
        return m_gyro;
    }

    public DifferentialDriveOdometry getOdometry() {
        return m_odometry;
    }

    public void periodic() {
        m_odometry.update(m_gyro.getRotation2d(), driveLeftEncoder.getDistance(), driveRightEncoder.getDistance());

        SmartDashboard.putNumber("Left encoder value meters", getLeftEncoderPosition());
        SmartDashboard.putNumber("RIGHT encoder value meters", getRightEncoderPosition());
        SmartDashboard.putNumber("Gyro heading", getHeading());
    }

}

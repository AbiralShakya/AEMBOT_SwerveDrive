package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.lib.PreferencesParser;
import frc.lib.controlsystems.SimPID;
import frc.lib.drive.SwerveParameters;
import frc.lib.logging.Logger;
import frc.lib.sensors.Potentiometer;

public class SwerveModule implements ISwerveModule{
    private final double kWheelDiamater; 
    private final int kEncoderResolution; 
    private final double potOffset;
    private final double gearRatio;
    private boolean isFieldCentric = false;
    private boolean stopped = false;

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final Potentiometer m_turningEncoder;

    private CANPIDController m_drivePIDController;

    private final SimPID m_turningPIDController;

}
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.kinematics.swerveModuleStates;

public interface ISwerveModule{
    public SwerveModuleState getState();

    public void setDesiredState(SwerveModuleState desiredState);

    public void getDesiredAngle();

    public double getDesiredSpeed();

    public double getSpeed();

    public double getAngleDegrees();

    public boolean isFieldCentric();

    public void stop();
}
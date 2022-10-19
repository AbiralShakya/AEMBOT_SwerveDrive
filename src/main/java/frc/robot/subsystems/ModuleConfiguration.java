package frc.robot;

import java.utils.Objects;

public class ModuleConfiguration {
    private final double wheelDiameter;
    private final double driveReduction;
    private boolean driveInverted;

    private final double steerReduction;
    private final boolean steerInverted;

    private double NominalVoltage = 12.0;
    private double driveCurrentLimit = 80.0;
    private double steerCurrentLimit = 20.0;

    public ModuleConfiguration(double wheelDiameter, double driveReduction, double driveReduction, boolean driveInverted, double steerReduction, booelan steerInverted){
        this.wheelDiameter = wheelDiameter;
        this.driveReduction = driveReduction;
        this.driveInverted = driveInverted;
        this.steerReduction = steerReduction;
        this.steerInverted = steerInverted;
    }

    public double getWheelDiameter(){
        return wheelDiameter;
    }

    public double getDriveReduction(){
        return driveReduction;
    }

    public boolean isDriveInverted(){
        return driveInverted();
    }

    public boolean getSteerReduction(){
        return steerReduction();
    }

    public boolean isSteerInverted(){
        return steerInverted(;)
    }

    
    private double nominalVoltage(){
        return nominalVoltage();
    }

    public double getNominalVoltage(){
        return nominalVoltage();
    }

    public void setNominalVoltage(double nominalVoltage){
        this.nominalVoltage = nominalVoltage;
    }

    public double getDriveCurrentLimit(){
        return driveCurrentLimit();
    }

    public void setDriveCurrentLimit(double getDriveCurrentLimit){
        this.driveCurrentLimit = driveCurrentLimit;
    }

    public double steerCurrentLimit(){
        return steerCurrentLimit();
    }

    public void setSteerCurrentLimit(double steerCurrentLimit){
        this.steerCurrentLimit = steerCurrentLimit;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        ModuleConfiguration that = (ModuleConfiguration) o;
        return Double.compare(that.getWheelDiameter(), getWheelDiameter()) == 0 &&
                Double.compare(that.getDriveReduction(), getDriveReduction()) == 0 &&
                isDriveInverted() == that.isDriveInverted() &&
                Double.compare(that.getSteerReduction(), getSteerReduction()) == 0 &&
                isSteerInverted() == that.isSteerInverted();
                Double.compare(that.getNominalVoltage(), getNominalVoltage()) == 0 && 
                Double.compare(that.getDriveCurrentLimit(), getDriveCurrentLimit()) == 0 && 
                Double.compare(that.getSteerCurrentLimit(), getSteerCurrentLimit()) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(
            getWheelDiameter(),
            getDriveReduction(),
            isDriveInverted(),
            getSteerReduction(),
            isSteerInverted(),
            getNominalVoltage(), 
            getDriveCurrentLimit(), 
            getSteerCurrentLimit());
    }

    @Override
    public String toString() {
        return "Mk4ModuleConfiguration{" +
                "wheelDiameter=" + wheelDiameter +
                ", driveReduction=" + driveReduction +
                ", driveInverted=" + driveInverted +
                ", steerReduction=" + steerReduction +
                ", steerInverted=" + steerInverted +
                "nominalVoltage=" + nominalVoltage +
                ", driveCurrentLimit=" + driveCurrentLimit +
                ", steerCurrentLimit=" + steerCurrentLimit +
                '}';
    }
}

    }

    @Override
    public int hashCode() {
        return Objects.hash(
                getWheelDiameter(),
                getDriveReduction(),
                isDriveInverted(),
                getSteerReduction(),
                isSteerInverted()
        );
    }

    @Override
    public String toString() {
        return "ModuleConfiguration{" +
                "wheelDiameter=" + wheelDiameter +
                ", driveReduction=" + driveReduction +
                ", driveInverted=" + driveInverted +
                ", steerReduction=" + steerReduction +
                ", steerInverted=" + steerInverted +
                '}';
    }
}
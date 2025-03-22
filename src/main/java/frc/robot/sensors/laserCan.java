package frc.robot.sensors;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import frc.robot.Constants.CoralIntakeConstants;

public class laserCan {
    public LaserCan laserCan;

    public laserCan(){
        laserCan = new LaserCan(CoralIntakeConstants.LASER_CAN);
        try {
            laserCan.setRangingMode(RangingMode.SHORT);
            laserCan.setRegionOfInterest(new RegionOfInterest(8, 8, 6, 6)); // center(x,y), scale(w,h)
            laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    public LaserCan getLaserCan(){
        return this.laserCan;
    }

    public boolean hasCoral(){
        System.out.println("LaserCan Distance: "+laserCan.getMeasurement().distance_mm);
        return (CoralIntakeConstants.THRESHOLD_MM < laserCan.getMeasurement().distance_mm);
    }

    public double getDistance(){
        return (laserCan.getMeasurement().distance_mm);
    }
}

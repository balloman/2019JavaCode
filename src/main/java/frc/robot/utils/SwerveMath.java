package frc.robot.utils;

import frc.robot.Constants;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static frc.robot.Constants.Wheel;
import static frc.robot.Constants.Wheel.*;

public class SwerveMath {
    private double x;
    private double y;
    private double z;
    private boolean autonomous;
    public boolean deadBandActive;
    private List<Double> inputs;
    private Map<String, Double> abcd;
    public Map<Wheel, Double> speeds;
    public Map<Wheel, Double> angles;

    public SwerveMath(double x, double y, double z, boolean autonomous) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.autonomous = autonomous;
        inputs = new ArrayList<>();
        inputs.add(x);
        inputs.add(y);
        inputs.add(z);
        deadBandActive = configDeadband();
        abcd = new HashMap<>();
    }

    public void calculate(double x, double y, double z, boolean autonomous){
        this.x = x;
        this.y = y;
        this.z = z;
        this.autonomous = autonomous;
        inputs.set(0, x);
        inputs.set(1, y);
        inputs.set(2, z);
        deadBandActive = configDeadband();
    }

    public SwerveMath(List<Double> inputs, boolean autonomous) {
        this.autonomous = autonomous;
        this.inputs = inputs;
        deadBandActive = configDeadband();
    }

    private boolean configDeadband(){
        if (!autonomous){
            for (int i = 0; i < inputs.size(); i++) {
                if (Math.abs(inputs.get(i)) < 0.13){
                    inputs.set(i, 0.0);
                }
            }
        }
        double sum = 0.0;
        for (var item :
                inputs) {
            sum += Math.abs(item);
        }
        return sum == 0;
    }

    /*
    Finds the abcd values
     */
    public Map<String, Double> configABCD(){
        Map<String, Double> abcd = new HashMap<>();
        double STR = x;
        double FWD = -y;
        double RCW = z;
        abcd.put("A", STR - RCW * Constants.WheelbaseConstant);
        abcd.put("B", STR + RCW * Constants.WheelbaseConstant);
        abcd.put("C", FWD - RCW * Constants.WheelbaseConstant);
        abcd.put("D", FWD + RCW * Constants.WheelbaseConstant);
        this.abcd = abcd;
        return abcd;
    }


    /*
    Finds the wheel speeds: FR, FL, RL, RR
     */
    public Map<Wheel, Double> calcSpeeds(Map<String, Double> abcd){
        Map<Wheel, Double> speeds = new HashMap<>();
        speeds.put(Wheel.FR, pythagoreanTheorem(abcd.get("B"), abcd.get("C")));
        speeds.put(Wheel.FL, pythagoreanTheorem(abcd.get("B"), abcd.get("D")));
        speeds.put(Wheel.BL, pythagoreanTheorem(abcd.get("A"), abcd.get("D")));
        speeds.put(Wheel.BR, pythagoreanTheorem(abcd.get("A"), abcd.get("C")));
        Normalize(speeds);
        this.speeds = speeds;
        return speeds;
    }

    /*
    Finds the wheel speeds: FR, FL, RL, RR
     */
    public Map<Wheel, Double> calcSpeeds(){
        Map<Wheel, Double> speeds = new HashMap<>();
        speeds.put(Wheel.FR, pythagoreanTheorem(abcd.get("B"), abcd.get("C")));
        speeds.put(Wheel.FL, pythagoreanTheorem(abcd.get("B"), abcd.get("D")));
        speeds.put(Wheel.BL, pythagoreanTheorem(abcd.get("A"), abcd.get("D")));
        speeds.put(Wheel.BR, pythagoreanTheorem(abcd.get("A"), abcd.get("C")));
        Normalize(speeds);
        this.speeds = speeds;
        return speeds;
    }

    /*
    Calculates the wheel angles
     */
    public Map<Wheel, Double> calcAngles(Map<String, Double> abcd){
        Map<Wheel, Double> angles = new HashMap<>();
        angles.put(FR, Math.atan2(abcd.get("B"), abcd.get("C")));
        angles.put(FL, Math.atan2(abcd.get("B"), abcd.get("D")));
        angles.put(BL, Math.atan2(abcd.get("A"), abcd.get("D")));
        angles.put(BR, Math.atan2(abcd.get("A"), abcd.get("C")));
        angles.replaceAll((k, v) -> Math.toDegrees(angles.get(k)));
        this.angles = angles;
        return angles;
    }

    /*
    Calculates the wheel angles
     */
    public Map<Wheel, Double> calcAngles(){
        Map<Wheel, Double> angles = new HashMap<>();
        angles.put(FR, Math.atan2(abcd.get("B"), abcd.get("C")));
        angles.put(FL, Math.atan2(abcd.get("B"), abcd.get("D")));
        angles.put(BL, Math.atan2(abcd.get("A"), abcd.get("D")));
        angles.put(BR, Math.atan2(abcd.get("A"), abcd.get("C")));
        angles.replaceAll((k, v) -> Math.toDegrees(angles.get(k)));
        this.angles = angles;
        return angles;
    }

    //------------------------------------------Math Utilities-------------------------------------------------

    public enum AngleRange{
        None,
        NegPi_Pi,
        Zero_2Pi
    }

    public enum AngleUnits{
        RadIn_RadOut,
        RadIn_DegOut,
        DegIn_DegOut,
        DegIn_RadOut
    }

    public static double pythagoreanTheorem(double a, double b){
        return Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
    }

    /*
    Wraps an angle
     */
    public static double wrapAngle(double angle, AngleRange range, AngleUnits units){
        var outAngle = 0.0;
        if (range == AngleRange.None) return angle;
        if (units == AngleUnits.DegIn_DegOut || units == AngleUnits.DegIn_RadOut) angle = Math.toRadians(angle);
        if (range == AngleRange.NegPi_Pi){
            angle = angle % (2*Math.PI);
            if (angle > Math.PI){
                outAngle = angle - (2*Math.PI);
            }else{
                outAngle = angle;
            }
        }else{
            outAngle = angle % (2*Math.PI);
        }
        if (units == AngleUnits.DegIn_DegOut || units == AngleUnits.RadIn_DegOut){
            return Math.toDegrees(outAngle);
        }
        return outAngle;
    }

    /*
    Finds the shortest path between an angle
     */
    public static double angleDistance(double currentAngle, double targetAngle){
        double diff = targetAngle - currentAngle;
        diff = diff % 360;
        return wrapAngle(diff, AngleRange.NegPi_Pi, AngleUnits.DegIn_DegOut);
    }

    public static void Normalize(Map<Wheel, Double> speeds){
        double maxSpeed = speeds.get(FL);
        for (Double speed :
                speeds.values()) {
            if (speed > maxSpeed) maxSpeed = speed;
        }
        if (maxSpeed > 1){
            speeds.put(FL, speeds.get(FL) / maxSpeed);
            speeds.put(FR, speeds.get(FR) / maxSpeed);
            speeds.put(BL, speeds.get(BL) / maxSpeed);
            speeds.put(BR, speeds.get(BR) / maxSpeed);
        }
    }

    public static double[] rotateHeading(double x, double y, double angle){
        angle = Math.toRadians(angle);
        double angleSin = Math.sin(angle);
        double angleCos = Math.cos(angle);
        double yFused = y * angleCos + x * angleSin;
        double xFused = x * angleCos - y * angleSin;
        return new double[]{xFused, yFused};
    }
}

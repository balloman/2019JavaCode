package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;
import frc.robot.Constants;
import frc.robot.utils.SwerveMath;

import static frc.robot.Constants.Wheel;
import static frc.robot.Constants.Wheel.*;

public class SwerveDriveSubsystem extends SubsystemBase {

    private Map<Wheel, TalonFX> driveMotors;
    private Map<Wheel, TalonFX> rotationMotors;
    private Map<Wheel, AnalogInput> rotationEncoders;
    private Map<Wheel, Double> offsets;
    private SwerveMath swerveMath;
    private PIDController rotationPid;

    private final static SwerveDriveSubsystem INSTANCE = new SwerveDriveSubsystem();


    private SwerveDriveSubsystem() {
        driveMotors = new HashMap<>();
        rotationMotors = new HashMap<>();
        rotationEncoders = new HashMap<>();

        setOffsets();

        driveMotors.put(BL, new TalonFX(0));
        driveMotors.put(FL, new TalonFX(1));
        driveMotors.put(FR, new TalonFX(2));
        driveMotors.put(BR, new TalonFX(3));

        rotationMotors.put(BL, new TalonFX(4));
        rotationMotors.put(FL, new TalonFX(5));
        rotationMotors.put(BR, new TalonFX(6));
        rotationMotors.put(FR, new TalonFX(7));

        rotationEncoders.put(FR, new AnalogInput(0));
        rotationEncoders.put(FL, new AnalogInput(1));
        rotationEncoders.put(BL, new AnalogInput(2));
        rotationEncoders.put(BR, new AnalogInput(3));

        for (var wheel :
                rotationMotors.keySet()) {
            rotationMotors.get(wheel).config_kP(0, 0.05);
            rotationMotors.get(wheel).getSensorCollection()
                    .setIntegratedSensorPosition(offsets.get(wheel), 100);//Set the encoder position to keep absolute
        }

        System.out.println("We are here"); //Debug message to alert that we are live with swerve code, noice
        swerveMath = new SwerveMath(0,0,0,false); //Instantiate the SwerveMath class
        rotationPid = new PIDController(0.016, 0, 0); //Does nothing


    }

    /**
     * Returns the Singleton instance of this SwerveDriveSubsystem. This static method
     * should be used -- {@code SwerveDriveSubsystem.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static SwerveDriveSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Gets the value for the rotation encoder
     * @param wheel Wheel enum
     * @return Double for the encoder value
     */
    public double getRotationEncoder(Wheel wheel){
        return (rotationMotors.get(wheel).getSensorCollection().getIntegratedSensorPosition())
                % 36864;
    }

    /**
     * Gets the refnums for the rotation motors
     * @return A TalonFX object
     */
    public Map<Wheel, TalonFX> getRefnums(){
        return rotationMotors;
    }

    /**
     * Gets the offsets
     * @return Map<Wheel, Double> of offsets
     */
    public Map<Wheel, Double> getOffsets() {
        return offsets;
    }

    /**
     * Sets a wheel to a specified speed
     * @param wheel The wheel enum
     * @param speed Speed value to set to -1 to 1
     */
    public void setDriveMotors(Wheel wheel, double speed){
        driveMotors.get(wheel).set(ControlMode.PercentOutput, speed);
    }

    /**
     * Sets the speed of a given rotational motor
     * @param wheel The wheel enum
     * @param speed Speed to set to -1 to 1
     */
    public void setRotationMotors(Wheel wheel, double speed){
        rotationMotors.get(wheel).set(ControlMode.PercentOutput, speed);
    }

    /**
     * Sets the position of a given rotational motor using the TalonFX Control Mode
     * @param wheel The wheel
     * @param position The position to go to
     */
    public void setRotationMotorsByPosition(Wheel wheel, double position){
        rotationMotors.get(wheel).set(TalonFXControlMode.Position, position);
    }

    /**
     *  Calibrates the wheels and records their offsets to offset.txt
     */
    public void calibrate(){
        Path file = Paths.get("/home/lvuser/offset.txt");
        //Just try to create a file and if we get the error that it already exists, we'll ignore it and leave the file be
        try {
            Files.createFile(file);
        }catch (IOException e){
            System.out.println(e.getMessage());
        }
        //Retrieve the encoder values and set the classwide offsets
        offsets.put(FL, (double) rotationEncoders.get(FL).getValue() * Constants.AbsToInt);
        offsets.put(FR, (double) rotationEncoders.get(FR).getValue() * Constants.AbsToInt);
        offsets.put(BL, (double) rotationEncoders.get(BL).getValue() * Constants.AbsToInt);
        offsets.put(BR, (double) rotationEncoders.get(BR).getValue() * Constants.AbsToInt);
        //Write these offsets to file
        File fout = new File(file.toString());
        try {
            BufferedWriter bw =
                    new BufferedWriter(new OutputStreamWriter(new FileOutputStream(fout)));
            bw.write(MapToString(offsets));
            bw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Sets the offsets to be used by the class to be retrieved from offset.txt
     */
    public void setOffsets(){
        Map<Wheel, Double> map = new HashMap<>();
        try {
            BufferedReader reader = new BufferedReader(new FileReader("/home/lvuser/offset.txt"));
            String line = reader.readLine();
            while (line != null){
                var pair = StringToMap(line);
                map.put(pair.getKey(), pair.getValue() * Constants.AbsToInt);
                line = reader.readLine();
            }
        } catch (IOException e) {
            map.put(FL, 0.0);
            map.put(FR, 0.0);
            map.put(BL, 0.0);
            map.put(BR, 0.0);
        }
        this.offsets = map;
    }

    /**
     * Calculates the angle and speed for a given wheel
     * @param wheel The wheel to find
     * @param math A SwerveMath object that already has the math applied
     * @return An array with 2 values, where the first is the angle, and the second is speed
     */
    public double[] wheelCalc(Wheel wheel, SwerveMath math){
        var encoder = getRotationEncoder(wheel); //Gets the encoder value
        var angle = math.angles.get(wheel); //Gets the angle
        var speed = math.speeds.get(wheel); //Gets the speed
        SmartDashboard.putNumber(wheel.name() + "Desired Angle", angle);
        SmartDashboard.putNumber(wheel.name(), encoder);

        //Get the current angle, convert it to degrees, and then wrap it
        var currentAngle = SwerveMath.wrapAngle((encoder * Constants.EncoderConstant),
                SwerveMath.AngleRange.NegPi_Pi, SwerveMath.AngleUnits.DegIn_DegOut);
        SmartDashboard.putNumber(wheel.name() + "CurrentAngle", currentAngle);

        //Find the distance and reverse if need be
        var angleDist = SwerveMath.angleDistance(currentAngle, angle);
        var altAngleDist = SwerveMath.angleDistance(currentAngle, angle+180);
        if (Math.abs(angleDist) > 90){
            return new double[]{altAngleDist * 102.4, -speed};
        }
        return new double[]{angleDist * 102.4, speed};
    }

    /**
     * Actually drives the robot lol
     * @param x the strafe value
     * @param y the forward value
     * @param z the rotation value
     * @param autonomous whether to run in autonomous or not (deadband)
     */
    public void Drive(double x, double y, double z, boolean autonomous){
        swerveMath.calculate(x, y, z, autonomous); //Input the swerveMath object with our variables
        swerveMath.configABCD(); // Do that weird abc math
        swerveMath.calcAngles(); //Find the angles
        swerveMath.calcSpeeds(); //Find the speeds

        //Loop over the wheels and set their speeds and position accordingly
        for(Wheel wheel: rotationMotors.keySet()){
            var results = wheelCalc(wheel, swerveMath);
            var distance = results[0];
            var speed = results[1];

            if (!swerveMath.deadBandActive){
                setRotationMotorsByPosition(wheel, distance+getRotationEncoder(wheel));
                setDriveMotors(wheel, 0);
            }else{
                setRotationMotors(wheel, 0);
                setDriveMotors(wheel, 0);
            }
        }
    }

    /**
     * Helper function to convert the map to a string
     * @param map The map to convert
     * @return A string in the format of 'Wheel Offset'\n
     */
    public static String MapToString(Map<Constants.Wheel, Double> map){
        StringBuilder output = new StringBuilder();
        for (Wheel wheel :
                map.keySet()) {
            output.append(wheel.name());
            output.append(" ");
            output.append(map.get(wheel));
            output.append("\n");
        }
        return output.toString();
    }

    /**
     * Converts a given string to an entry in a map
     * @param str The string line to convert
     * @return An entry
     */
    public static Map.Entry<Wheel, Double> StringToMap(String str){
        Wheel wheel;
        double iDouble;

        String[] splitStrings = str.split(" ");
        String wheelString = splitStrings[0];
        String doubleString = splitStrings[1];
        wheel = Wheel.valueOf(wheelString);
        iDouble = Double.parseDouble(doubleString);
        return new java.util.AbstractMap.SimpleEntry<>(wheel, iDouble);
    }

    /**
     * Returns the TalonFX drive encoder
     * @param wheel The wheel
     * @return A double of the
     */
    public double getTalonDriveEncoder(Wheel wheel){
        return driveMotors.get(wheel).getSensorCollection().getIntegratedSensorPosition();
    }

    /**
     * Runs sort of with periodic tasks
     */
    @Override
    public void periodic() {
        for (Wheel wheel :
                rotationEncoders.keySet()) {
            SmartDashboard.putNumber(wheel.name() + "Raw", rotationEncoders.get(wheel).getValue());
        }
    }
}


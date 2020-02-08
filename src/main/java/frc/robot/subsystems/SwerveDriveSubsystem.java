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
        for (var encoder :
                rotationEncoders.keySet()) {
            System.out.println("Analog Input created at: " + rotationEncoders.get(encoder).getChannel());
        }

        for (var wheel :
                rotationMotors.keySet()) {
            rotationMotors.get(wheel).config_kP(0, 0.05);
        }

        System.out.println("We are here");
        swerveMath = new SwerveMath(0,0,0,false);
        rotationPid = new PIDController(0.016, 0, 0);


    }

    /**
     * Returns the Singleton instance of this SwerveDriveSubsystem. This static method
     * should be used -- {@code SwerveDriveSubsystem.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static SwerveDriveSubsystem getInstance() {
        return INSTANCE;
    }

    public double getRotationEncoder(Wheel wheel){
        return (rotationMotors.get(wheel).getSensorCollection().getIntegratedSensorPosition())
                % 36864;
    }

    public Map<Wheel, TalonFX> getRefnums(){
        return rotationMotors;
    }

    public Map<Wheel, Double> getOffsets() {
        return offsets;
    }

    public void setDriveMotors(Wheel wheel, double speed){
        driveMotors.get(wheel).set(ControlMode.PercentOutput, speed);
    }

    /**
     *
     * @param wheel
     * @param speed
     */
    public void setRotationMotors(Wheel wheel, double speed){
        rotationMotors.get(wheel).set(ControlMode.PercentOutput, speed);
    }

    public void setRotationMotorsByPosition(Wheel wheel, double position){
        rotationMotors.get(wheel).set(TalonFXControlMode.Position, position);
    }

    public void calibrate(){
        Path file = Paths.get("/home/lvuser/offset.txt");
        try {
            Files.createFile(file);
        }catch (IOException e){
            System.out.println(e.getMessage());
        }
        offsets.put(FL, (double) rotationEncoders.get(FL).getValue());
        offsets.put(FR, (double) rotationEncoders.get(FR).getValue());
        offsets.put(BL, (double) rotationEncoders.get(BL).getValue());
        offsets.put(BR, (double) rotationEncoders.get(BR).getValue());
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

    public double[] wheelCalc(Wheel wheel, SwerveMath math){
        var encoder = getRotationEncoder(wheel); //Gets the encoder value
        var angle = math.angles.get(wheel); //Gets the angle
        var speed = math.speeds.get(wheel); //Gets the speed
        SmartDashboard.putNumber(wheel.name() + "Desired Angle", angle);
        SmartDashboard.putNumber(wheel.name(), encoder);

        var currentAngle = SwerveMath.wrapAngle((encoder * Constants.EncoderConstant),
                SwerveMath.AngleRange.NegPi_Pi, SwerveMath.AngleUnits.DegIn_DegOut);
        SmartDashboard.putNumber(wheel.name() + "CurrentAngle", currentAngle);

        var angleDist = SwerveMath.angleDistance(currentAngle, angle);
        var altAngleDist = SwerveMath.angleDistance(currentAngle, angle+180);
        if (Math.abs(angleDist) > 90){
            return new double[]{altAngleDist * 102.4, -speed};
        }
        return new double[]{angleDist * 102.4, speed};
    }

    public void Drive(double x, double y, double z, boolean autonomous){
        swerveMath.calculate(x, y, z, autonomous);
        swerveMath.configABCD();
        swerveMath.calcAngles();
        swerveMath.calcSpeeds();

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

    public static Map.Entry<Wheel, Double> StringToMap(String str){
        Wheel wheel;
        double iDouble;

        String[] splitStrings = str.split(" ");
        String wheelString = splitStrings[0];
        String doubleString = splitStrings[1];
        wheel = Wheel.valueOf(wheelString);
        iDouble = Double.parseDouble(doubleString);
        return new java.util.AbstractMap.SimpleEntry<Wheel, Double>(wheel, iDouble);
    }

    public double getTalonDriveEncoder(Wheel wheel){
        return driveMotors.get(wheel).getSensorCollection().getIntegratedSensorPosition();
    }

    @Override
    public void periodic() {
        for (Wheel wheel :
                rotationEncoders.keySet()) {
            SmartDashboard.putNumber(wheel.name() + "Raw", rotationEncoders.get(wheel).getValue());
        }
    }
}


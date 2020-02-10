package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavXSubsystem extends SubsystemBase {

// Any variables/fields used in the constructor must appear before the "INSTANCE" variable
// so that they are initialized before the constructor is called.

    private AHRS navX;

    /**
     * The Singleton instance of this NavXSubsystem. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static NavXSubsystem INSTANCE = new NavXSubsystem();

    /**
     * Creates a new instance of this NavXSubsystem.
     * This constructor is private since this class is a Singleton. External classes
     * should use the {@link #getInstance()} method to get the instance.
     */
    private NavXSubsystem() {
        navX = new AHRS(SPI.Port.kMXP);
    }

    /**
     * Returns the Singleton instance of this NavXSubsystem. This static method
     * should be used -- {@code NavXSubsystem.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static NavXSubsystem getInstance() {
        return INSTANCE;
    }

    public double getHeading(){
        return -navX.getYaw();
    }

    public void reset(){
        navX.reset();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("NavX", navX.getYaw());
    }
}


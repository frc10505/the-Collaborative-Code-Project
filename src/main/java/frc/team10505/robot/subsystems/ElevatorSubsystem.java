package frc.team10505.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    /* Variables */
    private final TalonFX elevatorMotor = new TalonFX(0);
    private double height = 0;

    private final PIDController controller = new PIDController(height, height, height);

    /* constructour */
    public ElevatorSubsystem() {

    }

    /* Commands for referance */
    public Command setHeight(double newHeight){
    return runOnce(() ->{
        height = newHeight;
    });
    }

    /* Calculations and such */

    @Override
    public void periodic() {

    }
}

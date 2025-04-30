package frc.team10505.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MonkeySubsystem extends SubsystemBase{
    /*Variables */
    private final TalonFX monkeyMotor;// = new TalonFX(782);
    private final SparkMax urahSpinnyMotor;// = new SparkMax(872, MotorType.kBrushless);

    private final DutyCycleEncoder monkeyEncoder = new DutyCycleEncoder(0);

    private final ElevatorFeedforward ffeController;// = new ElevatorFeedforward(0.1, 0.25, 0.2, 0.2);
    private final PIDController controller;// = new PIDController(1.0, 0, 0);

    private double setPoint = 0;



    /*Simulation variables */

    public MonkeySubsystem(){
        if(Utils.isSimulation()){
            monkeyMotor = new TalonFX(782);
            urahSpinnyMotor = new SparkMax(872, MotorType.kBrushless);

            ffeController = new ElevatorFeedforward(0.1, 0.25, 0.2, 0.2);
            controller = new PIDController(1.0, 0, 0);
        }else{
            monkeyMotor = new TalonFX(782);
            urahSpinnyMotor = new SparkMax(872, MotorType.kBrushless);

            ffeController = new ElevatorFeedforward(0.1, 0.25, 0.2, 0.2);
            controller = new PIDController(1.0, 0, 0);
        }
    }

    /*methods */
    /*Commands to reference */
    public Command setHeight(double newHeight){
        return runOnce(() ->{
            setPoint = newHeight;
        });
    }

    /*calculations */
    private double getEncoder(){
        if(Utils.isSimulation()){
            return 0;//TODO change
        } else {
            return monkeyEncoder.get() * 360;
        }
    }

    private double getEffort(){
        return controller.calculate(getEncoder(), setPoint) + ffeController.calculate(0);
    }

    @Override
    public void periodic(){
        monkeyMotor.setVoltage(getEffort());
    }
}

package frc.team10505.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

/*
 * 
 * 
 * 
 * THIS IS NOTTTTT ACCURATE TO THE ROBOT!!!!!!!!!!!!
 */
public class AlgaeSubsystem extends SubsystemBase {

    private final int pivotMotorId = 0;
    private final double startingAngle = 0.0;

    private final TalonFX pivotMotor;// = new TalonFX(pivotMotorId);

    private final PIDController pivotController;// = new PIDController(1.0, 0.0, 0.0);
    private final ArmFeedforward pivotFeedforward;// = new ArmFeedforward(0.3, 0.4, 0.2, 0.2);

    private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(1);

    private double setPoint = startingAngle;

    /*Simulation variables */
    private final Mechanism2d pivotMech = new Mechanism2d(1.5, 1.5);
    private MechanismRoot2d pivotRoot = pivotMech.getRoot("pivotRoot", 0.75, 0.75);
    private MechanismLigament2d pivotViz = pivotRoot.append(new MechanismLigament2d("pivotViz", 0.5, startingAngle));
    private SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getKrakenX60(2), 80, SingleJointedArmSim.estimateMOI(0.305, 2), 0.305, Units.degreesToRadians(-110), Units.degreesToRadians(110), true, Units.degreesToRadians(startingAngle));


    public AlgaeSubsystem(){
        if(Utils.isSimulation()){
            pivotMotor = new TalonFX(pivotMotorId);
            pivotController = new PIDController(1.0, 0.0, 0.0);
            pivotFeedforward = new ArmFeedforward(0.0, 0.4, 0.2, 0.2);
        }else{
            pivotMotor = new TalonFX(pivotMotorId, "kingCan");
            pivotController = new PIDController(1.0, 0.0, 0.0);
            pivotFeedforward = new ArmFeedforward(0.3, 0.4, 0.2, 0.2);
        }



    }

    public Command setAngle(double newAngle){
        return runOnce(() ->{
            setPoint = newAngle;
        });
    }


    public double calculateVoltage(){
        if(Utils.isSimulation()){
        return pivotFeedforward.calculate(Units.degreesToRadians(pivotViz.getAngle()), 0) + pivotController.calculate(pivotViz.getAngle(), setPoint);
        }else{
          return pivotFeedforward.calculate(pivotEncoder.get(), 0) + pivotController.calculate(pivotEncoder.get(), setPoint);
        }
    }


    @Override
    public void periodic(){

        pivotMotor.setVoltage(calculateVoltage());

        if(Utils.isSimulation()){
            pivotSim.setInput(pivotMotor.getMotorVoltage().getValueAsDouble());
            pivotSim.update(0.01);

            pivotViz.setAngle(new Rotation2d(Units.radiansToDegrees(pivotSim.getAngleRads())));
        }

        SmartDashboard.putNumber("setPoint", setPoint);
        SmartDashboard.putNumber("pivot encoder", pivotEncoder.get());
    }

}

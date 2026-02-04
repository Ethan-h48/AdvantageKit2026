package frc.robot.subsystems.Indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {

    private final SparkMax hopperMotor;
    private final SparkMax feederMotor;

    private SparkMaxConfig configHopper;
    private SparkMaxConfig configFeeder;


        public Indexer () {
            configFeeder = new SparkMaxConfig();
            configHopper = new SparkMaxConfig();

            configFeeder.idleMode(IdleMode.kCoast);
            configHopper.idleMode(IdleMode.kCoast);

            
            configFeeder.closedLoop.p(FeederConstants.p);
            configFeeder.closedLoop.p(FeederConstants.i);
            configFeeder.closedLoop.p(FeederConstants.d);
            
            configHopper.closedLoop.p(HopperConstants.p);
            configHopper.closedLoop.p(HopperConstants.i);
            configHopper.closedLoop.p(HopperConstants.d);

            hopperMotor = new SparkMax(IndexerConstants.hopperMotorCANID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
            feederMotor = new SparkMax(IndexerConstants.feederMotorCANID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

            feederMotor.configure(configFeeder, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            hopperMotor.configure(configHopper, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


        }

        public void stop(){
            hopperMotor.set(0);
            feederMotor.set(0);

        }

        //Puts balls into feeder
        public void runIndexer() {
            hopperMotor.set(IndexerConstants.hopperSpeed);
            feederMotor.set(IndexerConstants.feederSpeed);
        }

        public double getFeederSpeed(){
            return IndexerConstants.feederSpeed;
        }

        public double getHopperSpeed(){
            return IndexerConstants.hopperSpeed;
        }

}

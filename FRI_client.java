package application;


import java.util.concurrent.*;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.fri.*;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class FRI_client extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_7_R800_1;
	
	//FRI
	private FRIConfiguration configuration;
	private FRISession session;

	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_7_R800_1 = (LBR) getDevice(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_14_R820_1");
		
		
		System.out.println("Starting FRI...");
		
			}

	public void run() {
		
		/*FRIConfiguration*/
		//configuration = FRIConfiguration.createRemoteConfiguration(lbr_iiwa_7_R800_1, "172.31.1.146");
		//configuration.setSendPeriodMilliSec(5); //To review! modificato era a 10
        
		configuration = FRIConfiguration.createRemoteConfiguration(lbr_iiwa_7_R800_1, "192.170.10.146");
		configuration.setSendPeriodMilliSec(3); //To review! modificato era a 10 //040822 was 5 //31012023
		configuration.setReceiveMultiplier(1);
		
        getLogger().info("Creating FRI connection to " + configuration.getHostName());
        getLogger().info("SendPeriod: " + configuration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + configuration.getReceiveMultiplier());

		//configuration.setPortOnRemote(30200); //default PORT: 30200
		
		session = new FRISession(configuration);
        FRIJointOverlay jointOverlay = new FRIJointOverlay(session);

        //lbr_iiwa_7_R800_1.move(ptp(.0, .0, Math.toRadians(90), Math.toRadians(90), .0, .0, .0).setJointVelocityRel(0.1));
        //lbr_iiwa_7_R800_1.move(ptp(Math.toRadians(130), .0, .0, Math.toRadians(90), .0, Math.toRadians(-90), .0).setJointVelocityRel(0.1));
        //lbr_iiwa_7_R800_1.move(ptp(Math.toRadians(50), .0, .0, Math.toRadians(84), Math.toRadians(-127), Math.toRadians(-71), Math.toRadians(90)).setJointVelocityRel(0.1));
        //lbr_iiwa_7_R800_1.move(ptp(Math.toRadians(45), .0, .0, Math.toRadians(90), Math.toRadians(-90), Math.toRadians(-90), Math.toRadians(-90)).setJointVelocityRel(0.1));
        lbr_iiwa_7_R800_1.move(ptp(Math.toRadians(46.6), Math.toRadians(-6.88), .0, Math.toRadians(82.7), Math.toRadians(-90), Math.toRadians(-91.6), Math.toRadians(-90.44)).setJointVelocityRel(0.1));
        
        
        
       
        
        //getLogger().info("home reached.");
		
        try
        {
        	session.await(10, TimeUnit.SECONDS);
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            session.close();
            return;
        }
               
        getLogger().info("FRI connection established.");

        
        
        //PositionControlMode ctrMode = new PositionControlMode();
        
        CartesianImpedanceControlMode ctrMode = 	new CartesianImpedanceControlMode();
        //19/12/2022
        ctrMode.parametrize(CartDOF.X).setStiffness(100); //100
        ctrMode.parametrize(CartDOF.X).setDamping(0.1);
        
        
        
        ctrMode.parametrize(CartDOF.Y).setStiffness(1000); //800
        
        ctrMode.parametrize(CartDOF.Z).setStiffness(1000); //800
        
        //ctrMode.parametrize(CartDOF.A).setStiffness(150);
        //ctrMode.parametrize(CartDOF.B).setStiffness(150);
        //ctrMode.parametrize(CartDOF.C).setStiffness(150);
        
        
        //JointImpedanceControlMode ctrMode = new JointImpedanceControlMode(2000, 2000, 2000, 2000, 2000, 2000, 2000); //before 700
        //ctrMode.setDamping(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
        
        //PositionHold posHold = new PositionHold(ctrMode, -1, TimeUnit.SECONDS);
        PositionHold posHold = new PositionHold(ctrMode, -1, TimeUnit.SECONDS);
        
        try{
        	lbr_iiwa_7_R800_1.move(posHold.addMotionOverlay(jointOverlay));
        	
        } catch (final CommandInvalidException e) {
        	
        	getLogger().error(e.getLocalizedMessage());
        
        }
       
       		
		session.close();
		
		System.out.println("FRI exited");
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		FRI_client app = new FRI_client();
		app.runApplication();
	}
}

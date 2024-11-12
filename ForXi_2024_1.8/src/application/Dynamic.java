package application;


import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import javax.inject.Inject;
import javax.inject.Named;

import application.Dynamic.GetToolForce;
import application.Dynamic.robotmove;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.uiModel.IApplicationUI;
import com.kuka.task.ITaskLogger;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.med.deviceModel.LBRMed;

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
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class Dynamic extends RoboticsAPIApplication {
	@Inject
	private  LBR lbr;
	@Named("ToolTest")
	@Inject
	com.kuka.roboticsAPI.geometricModel.Tool needle_ToolTest;
//	@Inject
//	AbstractFrame TCP;
    @Inject
    private ITaskLogger logger;
    private IApplicationUI appUI;
 
    
    
    private double Fx;
	private double Fy;
	private double Fz;
	private double Fa;
	private double Fb;
	private double Fc;
    private double fx;
	private double fy;
	private double fz;
	private double fa;
	private double fb;
	private double fc;
	
	private double Mg;
	private double CenterofMassX;
	private double CenterofMassY;
	private double CenterofMassZ;
	private int count = 0;
	private boolean moveing = false;
	
	private JointPosition jointPos;
	private JointPosition jointPos_1;
	private JointPosition jointPos_2;
	private JointPosition jointPos_3;
	private JointPosition jointPos_4;
	private JointPosition jointPos_5;
	private JointPosition jointPos_6;
	private JointPosition jointPos_7;
	private JointPosition jointPos_8;
	
	
	
	
     public void  initialize(){
    	 needle_ToolTest.attachTo(lbr.getFlange());
    	 moveing = false;
    	 jointPos=new JointPosition(   Math.toRadians(40.2),
 				Math.toRadians(-13.02),
 				Math.toRadians(-2.20),
 				Math.toRadians(78.93),
 				Math.toRadians(47.49),
 				Math.toRadians(48.25),
 				Math.toRadians(-113.84));

 		jointPos_1=new JointPosition(   Math.toRadians(-55.87),
 				Math.toRadians(-11.16),
 				Math.toRadians(20.03),
 				Math.toRadians(98.11),
 				Math.toRadians(-39.44),
 				Math.toRadians(61.22),
 				Math.toRadians(-51.51));

 		jointPos_2=new JointPosition(   Math.toRadians(38.24),
 				Math.toRadians(-26.47),
 				Math.toRadians(20.03),
 				Math.toRadians(81.03),
 				Math.toRadians(39.50),
 				Math.toRadians(72.48),
 				Math.toRadians(-149.59));
 		jointPos_3=new JointPosition(   Math.toRadians(-14.63),
 				Math.toRadians(20.87),
 				Math.toRadians(20.03),
 				Math.toRadians(103.00),
 				Math.toRadians(14.32),
 				Math.toRadians(25.19),
 				Math.toRadians(-100.02));
 		jointPos_4=new JointPosition(   Math.toRadians(-7.87),
 				Math.toRadians(-28.71),
 				Math.toRadians(20.03),
 				Math.toRadians(108.30),
 				Math.toRadians(2.21),
 				Math.toRadians(78.09),
 				Math.toRadians(-104.49));
 		jointPos_5=new JointPosition(   Math.toRadians(-4.78),
 				Math.toRadians(-23.38),
 				Math.toRadians(20.03),
 				Math.toRadians(62.05),
 				Math.toRadians(13.46),
 				Math.toRadians(27.98),
 				Math.toRadians(-116.65));
 		jointPos_6=new JointPosition(   Math.toRadians(-9.76),
 				Math.toRadians(4.65),
 				Math.toRadians(20.03),
 				Math.toRadians(112.37),
 				Math.toRadians(10.91),
 				Math.toRadians(50.26),
 				Math.toRadians(-101.86));
 		jointPos_7=new JointPosition(   Math.toRadians(39.66),
 				Math.toRadians(-30.86),
 				Math.toRadians(7.27),
 				Math.toRadians(51.84),
 				Math.toRadians(49.63),
 				Math.toRadians(44.96),
 				Math.toRadians(-110.31));
 		jointPos_8=new JointPosition(   Math.toRadians(-0.25),
 				Math.toRadians(-23.97),
 				Math.toRadians(-54.95),
 				Math.toRadians(99.25),
 				Math.toRadians(-30.84),
 				Math.toRadians(75.37),
 				Math.toRadians(5.54));
    	 
	    }
     
     
     public void run() {
 		
    	ExecutorService executor = Executors.newCachedThreadPool();
 		Future<String> Move = executor.submit(new robotmove());
 		Future<String> ReviceForce = executor.submit(new GetToolForce());
 		
 	
 	
 		try {
 			String resultWithMove = Move.get();
 			
            logger.info("Runnable Result With Send: " + resultWithMove);

 		}
 		catch(InterruptedException e){
 			 e.printStackTrace();
 	}
 		catch(ExecutionException e){
 			 e.printStackTrace();
 		}
 		
 		
 		try {
            Thread.sleep(2000); 
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

 		
 	}
     
     
     public  class robotmove implements Callable<String> {

    	 
    	 public String call(){
    		 
    		 
    		 
    		 moveing =true;
    		 lbr.move(new PTP(jointPos).setJointVelocityRel(0.2));
    		 lbr.move(new PTP(jointPos_1).setJointVelocityRel(0.2));
    		 lbr.move(new PTP(jointPos_2).setJointVelocityRel(0.2));
    		 lbr.move(new PTP(jointPos_3).setJointVelocityRel(0.2));
    		 lbr.move(new PTP(jointPos_4).setJointVelocityRel(0.2));
    		 lbr.move(new PTP(jointPos_5).setJointVelocityRel(0.2));
    		 lbr.move(new PTP(jointPos_6).setJointVelocityRel(0.2));
    		 lbr.move(new PTP(jointPos_7).setJointVelocityRel(0.2));
    		 lbr.move(new PTP(jointPos_8).setJointVelocityRel(0.2));
    		 
    		 moveing = false;
    		 
    		 return "Move completed";
    		 
    	 }
    	 
     }
     public  class GetToolForce implements Callable<String>{

    	 public String call(){
    		 
    		 while (moveing){
    			 Fx = lbr.getExternalForceTorque(needle_ToolTest.getFrame("/TCP")).getForce().getX();
        		 
        		 Fy = lbr.getExternalForceTorque(needle_ToolTest.getFrame("/TCP")).getForce().getY();
        		 
        		 Fz = lbr.getExternalForceTorque(needle_ToolTest.getFrame("/TCP")).getForce().getZ();
        		 
        		 Fa = lbr.getExternalForceTorque(needle_ToolTest.getFrame("/TCP")).getTorque().getZ();
        		 
        		 Fb = lbr.getExternalForceTorque(needle_ToolTest.getFrame("/TCP")).getTorque().getY();
        		 
        		 Fc = lbr.getExternalForceTorque(needle_ToolTest.getFrame("/TCP")).getTorque().getX(); 
        		 
                 fx += Fx;
                 fy += Fy;
                 fz += Fz;
                 fa += Fa;
                 fb += Fb;
                 fc += Fc;
                         		 
        		 count++;
        		 
        		 try {
        	            Thread.sleep(200); 
        	        } catch (InterruptedException e) {
        	            e.printStackTrace();
        	        }

    		 }
    		 
    		 fx /= count;
    		 fy /= count;
    		 fz /= count;
    		 fa /= count;
    		 fb /= count;
    		 fc /= count;
  		 
    		 Mg = Math.sqrt(Math.pow(fx, 2)+Math.pow(fy, 2)+Math.pow(fz, 2))/9.8;
    		 
    		 CenterofMassX = fc/fx;
    		 CenterofMassY = fb/fy;
    		 CenterofMassZ = fa/fz;
    		 
    		 logger.info(" Mg:"+Mg+ " Kg");
    		 
      		 logger.info("Center of MassX:"+CenterofMassX);
    		 logger.info("Center of MassY:"+CenterofMassY);
    		 logger.info("Center of MassZ:"+CenterofMassZ);
    		 
			return "Force calculation completed";   	

	
         }
	
    	 
     }
}
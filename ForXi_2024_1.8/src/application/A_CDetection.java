package application;


//import applications.AppMainMenu;

import javax.inject.Inject;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.ObserverManager;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.MotionBatch;

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
public class A_CDetection extends RoboticsAPIApplication {
	
	private Controller kuka_Sunrise_Cabinet_1;
	@Inject
	private LBR _lbr;
	private ObserverManager obsM1;
    private C_Detection A; 
      
    
	@Override
	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		
		obsM1 = new ObserverManager();
				
		
		A = new C_Detection(_lbr, this,obsM1);
	
		
	}

	@Override
	public void run() {
		
		
		A.cd_enable();
		
		while(true){
			A.cd_enable();
		_lbr.move(ptp(getApplicationData().getFrame("/start")).setJointVelocityRel(0.3));
		MotionBatch ns = new MotionBatch(
				lin(getFrame("/start")).setCartVelocity(100),
				linRel(0, 0, 200).setCartVelocity(100),//x direction
				
				linRel(-150, 0, 0).setCartVelocity(100),//x direction 
							
				linRel(0, 150, 0).setCartVelocity(100),//y direction
				linRel(0, -150, 0).setCartVelocity(100)//y direction
				);
		_lbr.move(ns);
		}
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		A_CDetection app = new A_CDetection();
		app.runApplication();
	}
}

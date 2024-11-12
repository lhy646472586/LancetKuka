package application;


import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.HRCMotions.*;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;


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
public class HandGuidingDemo extends RoboticsAPIApplication {
	@Inject
	private LBR robot;
	//constants for limitation: max and min
	double[] jointLimitsMax = {
			1.407, 0.872, 0.087, -0.785, 0.087, 1.571, 0.087
	};
	double[] jointLimitsMin = {
			-1.407, 0.175, -0.087, -1.571, -0.087, -1.571, -0.087,
	};

	
	@Named("Tool_2")
	@Inject
	Tool needle;
	
	public HandGuidingMotion createhandGuidingMotion(){
		
//		HandGuidingMotion motion = new HandGuidingMotion();
//		motion.setJointVelocityLimit(1)
//		.setCartVelocityLimit(1000.0).setJointLimitViolationFreezesAll(false);
//		return motion;
		HandGuidingMotion motion = new HandGuidingMotion();
		motion.setJointVelocityLimit(1)
		.setCartVelocityLimit(1000.0).setJointLimitViolationFreezesAll(false);
		return motion;
	}
	public CartesianImpedanceControlMode createCartImp(){
		CartesianImpedanceControlMode mode = new CartesianImpedanceControlMode();
		mode.parametrize(CartDOF.ROT).setStiffness(300);
		mode.parametrize(CartDOF.X).setStiffness(1);
		mode.parametrize(CartDOF.Y).setStiffness(5000);
		mode.parametrize(CartDOF.Z).setStiffness(5000);
	
		return mode;
	}
	@Override
	public void initialize() {
		
		needle.attachTo(robot.getFlange());
		
//		robot.setHomePosition(
//				new JointPosition(0, 0, Math.toRadians(110), Math.toRadians(90), 0, Math.toRadians(-90), 0));
//		robot.move(ptpHome().setJointVelocityRel(0.2));
		
		Frame Ptest1= getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();


		needle.move(ptp(Ptest1).setBlendingCart(0).setJointVelocityRel(0.2));
		
	}

	@Override
	public void run() {
		
		// your application execution starts here
		getLogger().info("please use handGuiding to move needle to the place " +
				"you want to insert into");
		
		while (true){
		needle.getFrame("/tcp_2").move(createhandGuidingMotion());
		ThreadUtil.milliSleep(100);
		}
		
//		while (true){
////		needle.getFrame("/tcp_2").move(createhandGuidingMotion());
//		ThreadUtil.milliSleep(1000);
//		}
		
//		getLogger().info("guide needle to insert");
//		
//		needle.getFrame("/tcp_2").move(positionHold(createCartImp(), 50, TimeUnit.SECONDS));
//		
//		getLogger().info("Hand-Guiding-demonstration finished!");
	}
}
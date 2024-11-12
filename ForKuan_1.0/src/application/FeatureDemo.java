package application;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MytestIOIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.CartPlane;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.ioModel.Input;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.generated.ioAccess.SafeDataIOGroup;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

/**
 * @author micheler
 * 
 *
 */
public class FeatureDemo extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	@Inject

	private LBR lbr;
	@Named("gripper")
	@Inject
	private Tool gripper;
	
	@Named("Tool_2")
	@Inject
	private Tool needle;
	
	private ObjectFrame tcp;
	private ObjectFrame tcp_2;
	private Frame startpos;
	private Tool tool;
	
	private CartesianImpedanceControlMode impedanceMode = new CartesianImpedanceControlMode();
	private CartesianImpedanceControlMode _nullspacemode ;
	private JointPosition startjoint;
	private Frame cutFrame;
	private SafeDataIOGroup Test;
	public void initialize() {

		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
        tool = createFromTemplate("Tool_2");
	    tool.attachTo(lbr.getFlange());
	    
//		tcp_2=Tool_2.getFrame("/tcp_2");
//		Tool_2.attachTo(lbr.getFlange());
		needle.attachTo(lbr.getFlange());
		
		tcp=gripper.getFrame("/tcp");
		gripper.attachTo(lbr.getFlange());
		
		Test = new SafeDataIOGroup(kuka_Sunrise_Cabinet_1);
	}

	public void run() {
			
//		JointPosition actPos = lbr.getCurrentJointPosition();
//		//tcp.move(ptp(getApplicationData().getFrame("/P2")).setJointVelocityRel(.3));
//		Frame Ptest1= getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
//		Frame Ptest2 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy().transform((Transformation.ofTranslation(0, 200, 0)));
//		Frame Ptest3 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy().transform((Transformation.ofTranslation(-200, 200, 0)));
//		Frame Ptest4 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy().transform((Transformation.ofTranslation(-200, 0, 0)));
//		Frame Ptest5 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy().transform((Transformation.ofTranslation(0, 0, 200)));
//		Frame Ptest6 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy().transform((Transformation.ofTranslation(0, 200, 200)));
//		Frame Ptest7 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy().transform((Transformation.ofTranslation(-200, 200, 200)));
//		Frame Ptest8 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy().transform((Transformation.ofTranslation(-200, 0, 200)));
//		
//		ThreadUtil.milliSleep(5000);//frequency of recording
//		tcp.move(ptp(Ptest1).setBlendingCart(0).setJointVelocityRel(0.2));
//		
//		boolean absd=Test.getInput1();
//		int sel = 10;
//		while (sel != 8) {
//			sel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,
//				"Select Application? (ATTENTION: Robot will move!)",
//				"Startposition", // 0
//				"cartesian movement", // 1
//				"kinematic redundancy_1", // 2
//				"kinematic redundancy_2", // 3
//				"impedance control mode", // 4
//				"collision detection", // 5
//				"demo in loop", // 6
//				"Teaching by demonstration", // 7
//				"Exit application");//8
//			
//			switch(3) {
//				case 0:
////					startpos = lbr.getForwardKinematic(joints)
//					lbr.move(ptp(getFrame("/P2")).setJointVelocityRel(0.3).setMode(impedanceMode));
//			
//					break;
//				case 1:
//					moveCartesian(2);
//					break;
//				case 2:
//					moveNullspace(2);
//					break;
//				case 3:
//					moveNullspace_Positionhold();
//					break;
//				case 4:
//					stiffness();
//					break;
//				case 5:
//					moveNullspace_Positionhold_1();
//					break;
//				case 6:
//					
//					break;
//				case 7:
//					teaching();
//					break;
//				default:
//					//lbr.move(ptp(getFrame("/P2")).setJointVelocityRel(0.3));
//					tcp.move(lin(Ptest1).setBlendingCart(0).setJointVelocityRel(0.2));
//					break;			
//			}
//		}
		while (true){

			boolean btest=Test.getInput1();
			System.out.println("Input1:"+btest);


			btest=Test.getInput4();
			System.out.println("Input4:"+btest);
			Input btest1=Test.getInput("Input4");
			System.out.println("Input4_1:"+btest1);
			btest1=Test.getInput("Input1");
			System.out.println("Input1_1:"+btest1);
			ThreadUtil.milliSleep(3000);
		}
	}

	private void moveCartesian(int runs) {
		
		getLogger().info("moveCartesian");

		//collision condition definition
		ICondition forceCon = defineSensitivity();

		//motion programming
		lbr.move(ptp(getFrame("/P2")).setJointVelocityRel(0.3));
		MotionBatch cart = new MotionBatch(
			lin(getFrame("/P2")).setCartVelocity(100),
			linRel(-150, 0, 0).setCartVelocity(100),//x direction 
			lin(getFrame("/P2")).setCartVelocity(100),
			linRel(0, 150, 0).setCartVelocity(100),//y direction
			lin(getFrame("/P2")).setCartVelocity(100),
			linRel(0, 0, 150).setCartVelocity(100),//z direction
			lin(getFrame("/P2")).setCartVelocity(100))
		.breakWhen(forceCon);
		
		IMotionContainer motion;
		for (int i = 0; i < runs; i++) {
			motion = lbr.move(cart);
			if (motion.hasFired(forceCon)) {
				//Reaction after collision
				boolean resumeMotion = behaviourAfterCollision();
				if (!resumeMotion) break;
			}
		}
		
	}
	
	private void moveNullspace(int runs) {
		getLogger().info("Nullspace Movement");
		
		_nullspacemode = new CartesianImpedanceControlMode();
		_nullspacemode.parametrize(CartDOF.TRANSL).setStiffness(5000);
		_nullspacemode.parametrize(CartDOF.ROT).setStiffness(1.0);
		_nullspacemode.setDampingToDefaultValue();
		_nullspacemode.setNullSpaceStiffness(0.2);
		_nullspacemode.setNullSpaceDamping(0.7);
		_nullspacemode.setMaxJointSpeed(2.0, 
									   2.0, 
									   2.0, 
									   2.0, 
									   2.0, 
									   2.0, 
									   2.0);
		
	
		
		//collision condition definition
		ICondition forceCon = defineSensitivity();

		//motion programming
		lbr.move(ptp(getFrame("/P2")).setJointVelocityRel(.3));
		MotionBatch ns = new MotionBatch(
				lin(getFrame("/P2")).setCartVelocity(100),
				linRel(0, 0, 200).setCartVelocity(100),//x direction
				
				linRel(-150, 0, 0).setCartVelocity(100),//x direction 
							
				linRel(0, 150, 0).setCartVelocity(100),//y direction
				linRel(0, -150, 0).setCartVelocity(100)//y direction
				
			
				).setMode(_nullspacemode);
		
		for (int i = 0; i < runs; i++) {
		     lbr.move(ns);
		}
	}
	
	
	
	private void moveNullspace_Positionhold() {
		getLogger().info("Nullspace Movement in Positionhold");
		int answer;
		_nullspacemode = new CartesianImpedanceControlMode();
		_nullspacemode.parametrize(CartDOF.ALL).setDamping(0.5);
		_nullspacemode.parametrize(CartDOF.X).setStiffness(1);
		_nullspacemode.parametrize(CartDOF.Y).setStiffness(1);
		_nullspacemode.parametrize(CartDOF.Z).setStiffness(1);
		_nullspacemode.parametrize(CartDOF.A).setStiffness(1.0);
		_nullspacemode.parametrize(CartDOF.B).setStiffness(1.0);
		_nullspacemode.parametrize(CartDOF.C).setStiffness(1.0);
		_nullspacemode.setDampingToDefaultValue();
		_nullspacemode.setNullSpaceStiffness(0.2);
		_nullspacemode.setNullSpaceDamping(0.7);
		_nullspacemode.setMaxJointSpeed(2.0, 
									   2.0, 
									   2.0, 
									   2.0, 
									   2.0, 
									   2.0, 
									   2.0);
		
		//motion programming
		//tcp.move(ptp(getApplicationData().getFrame("/P2")).setJointVelocityRel(.3));
		
		IMotionContainer handle;
		//handle = gripper.getDefaultMotionFrame().moveAsync(positionHold(_nullspacemode, -1, TimeUnit.SECONDS));
		handle=needle.moveAsync(positionHold(_nullspacemode, -1, TimeUnit.SECONDS));
		
		answer = getApplicationUI().displayModalDialog(
				ApplicationDialogType.INFORMATION,"Move to Startposition and End", "LowStiffness", "MidStiffness", "HighStiffness");
		
		
		
		handle.cancel();
		
	}
	
	private void moveNullspace_Positionhold_1() {
		getLogger().info("Nullspace Movement in Positionhold");
		int answer;
		_nullspacemode = new CartesianImpedanceControlMode();
		_nullspacemode.parametrize(CartDOF.TRANSL).setStiffness(30);
		_nullspacemode.parametrize(CartDOF.X).setStiffness(10);
		_nullspacemode.parametrize(CartDOF.Y).setStiffness(10);
		_nullspacemode.parametrize(CartDOF.Z).setStiffness(10);
		_nullspacemode.parametrize(CartDOF.A).setStiffness(1.0);
		_nullspacemode.parametrize(CartDOF.B).setStiffness(1.0);
		_nullspacemode.parametrize(CartDOF.C).setStiffness(1.0);
		_nullspacemode.setDampingToDefaultValue();
		_nullspacemode.setNullSpaceStiffness(0.2);
		_nullspacemode.setNullSpaceDamping(0.7);
		_nullspacemode.setMaxJointSpeed(2.0, 
									   2.0, 
									   2.0, 
									   2.0, 
									   2.0, 
									   2.0, 
									   2.0);
		
		//motion programming
		//tcp_2.move(ptp(getApplicationData().getFrame("/P2")).setJointVelocityRel(.3));
		
		IMotionContainer handle;
		//handle = gripper.getDefaultMotionFrame().moveAsync(positionHold(_nullspacemode, -1, TimeUnit.SECONDS));
		handle=tcp_2.moveAsync(positionHold(_nullspacemode, -1, TimeUnit.SECONDS));
		
		answer = getApplicationUI().displayModalDialog(
				ApplicationDialogType.INFORMATION,"Move to Startposition and End", "LowStiffness", "MidStiffness", "HighStiffness");
		
		
		
		handle.cancel();
		
	}
	
	
	private void stiffness() {
		getLogger().info("Stiffness");
		
		int answer = 0;
		double stiffX = 1000.0;
		double stiffY = 300.0;
		double stiffZ = 600.0;

		lbr.move(ptp(getFrame("/P2")).setJointVelocityRel(0.3));
		CartesianImpedanceControlMode modeHandfuehren = new CartesianImpedanceControlMode();
		
		do {
			switch (answer) {
			case 0:
				stiffX = 1000.0;
				stiffY = 300.0;
				stiffZ = 600.0;
				break;
			case 1:
				stiffX = 100;
				stiffY = 100;
				stiffZ = 100;
				break;
			case 2:
				stiffX = 1000;
				stiffY = 1000;
				stiffZ = 1000;
				break;
			case 3:
				stiffX = 3000;
				stiffY = 3000;
				stiffZ = 3000;
				break;
			}
			modeHandfuehren.parametrize(CartDOF.X).setStiffness(stiffX);
			modeHandfuehren.parametrize(CartDOF.Y).setStiffness(stiffY);
			modeHandfuehren.parametrize(CartDOF.Z).setStiffness(stiffZ);
			modeHandfuehren.parametrize(CartDOF.ROT).setStiffness(100.0);

			IMotionContainer handle;
				handle = tool.moveAsync(positionHold(modeHandfuehren, -1, TimeUnit.SECONDS));

				answer = getApplicationUI().displayModalDialog(
						ApplicationDialogType.INFORMATION,
						"Stiffness in X: " + stiffX
						+ " N/m\nStiffness in Y: " + stiffY
								+ " N/m\nStiffness in Z: " + stiffZ + " N/m",
						"Move to Startposition and End", "LowStiffness", "MidStiffness", "HighStiffness");
				handle.cancel();
				lbr.move(ptp(getFrame("/start")).setJointVelocityRel(0.3));
		} while (answer != 0);
	}	
	
	
	

	private void teaching(){
		CartesianImpedanceControlMode mode = new CartesianImpedanceControlMode();
		mode.parametrize(CartDOF.ALL).setStiffness(80);
		mode.parametrize(CartDOF.ROT).setStiffness(10);
	
		ArrayList<JointPosition> positions = new ArrayList<JointPosition>();
		
		JointTorqueCondition fc = new JointTorqueCondition(JointEnum.J1, -15, 15);
		
		lbr .move(ptp(getApplicationData().getFrame("/start")));
		
		IMotionContainer posHold = lbr .moveAsync(positionHold(mode,8, TimeUnit.SECONDS));//time of recording

		while(!posHold.isFinished()){
			ThreadUtil.milliSleep(100);//frequency of recording
			positions.add(lbr .getCurrentJointPosition());
		}
		
		ThreadUtil.milliSleep(1000);
				
		getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION, "Play recorded path", "Play");
		int sel = 10;
		while (sel !=1){
			lbr .move(ptp(positions.get(0)).setJointVelocityRel(.2));
	
			
			for(int i = 1; i < positions.size(); i++){
				lbr .moveAsync(ptp(positions.get(i)).setBlendingRel(0.5).setJointVelocityRel(0.3));
			}
			sel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Replay or End?", "Replay", "End");
		}
				
		lbr .move(ptp(positions.get(0)).setJointVelocityRel(.1));
	}
	
	private ICondition defineSensitivity() {
		double sensCLS = 10;
		getLogger().info("Sensitive in each axis: " +sensCLS + " Nm\nShow current torque processdata in each axis.");
		
		//Offset compensation
		double actTJ1 = lbr .getExternalTorque().getSingleTorqueValue(JointEnum.J1);
		double actTJ2 = lbr .getExternalTorque().getSingleTorqueValue(JointEnum.J2);
		double actTJ3 = lbr .getExternalTorque().getSingleTorqueValue(JointEnum.J3);
		double actTJ4 = lbr .getExternalTorque().getSingleTorqueValue(JointEnum.J4);
		double actTJ5 = lbr .getExternalTorque().getSingleTorqueValue(JointEnum.J5);
		double actTJ6 = lbr .getExternalTorque().getSingleTorqueValue(JointEnum.J6);
		double actTJ7 = lbr .getExternalTorque().getSingleTorqueValue(JointEnum.J7);
		
		getLogger().info("OffsetValue\nJ1 " + actTJ1 + "Nm\nJ2 " + actTJ2 + "Nm\nJ3 " + actTJ3 + "Nm\nJ4 " + actTJ4 + "Nm\nJ5 " + actTJ5 + "Nm\nJ6 " + actTJ6 + "Nm\nJ7 " + actTJ7 + "Nm");
		
		//condition on each axis
		JointTorqueCondition jt1 = new JointTorqueCondition(JointEnum.J1, -sensCLS+actTJ1, sensCLS+actTJ1);
		JointTorqueCondition jt2 = new JointTorqueCondition(JointEnum.J2, -sensCLS+actTJ2, sensCLS+actTJ2);
		JointTorqueCondition jt3 = new JointTorqueCondition(JointEnum.J3, -sensCLS+actTJ3, sensCLS+actTJ3);
		JointTorqueCondition jt4 = new JointTorqueCondition(JointEnum.J4, -sensCLS+actTJ4, sensCLS+actTJ4);
		JointTorqueCondition jt5 = new JointTorqueCondition(JointEnum.J5, -sensCLS+actTJ5, sensCLS+actTJ5);
		JointTorqueCondition jt6 = new JointTorqueCondition(JointEnum.J6, -sensCLS+actTJ6, sensCLS+actTJ6);
		JointTorqueCondition jt7 = new JointTorqueCondition(JointEnum.J7, -sensCLS+actTJ7, sensCLS+actTJ7);

		ICondition forceCon = jt1.or(jt2, jt3, jt4, jt5, jt6, jt7);
		return forceCon;
	}

	private boolean behaviourAfterCollision() {
		boolean resumeMotion = true; 
		int sel = 0;
		IMotionContainer handle;
		
		CartesianImpedanceControlMode soft = new CartesianImpedanceControlMode();
			soft.parametrize(CartDOF.ALL).setDamping(.7);
			soft.parametrize(CartDOF.ROT).setStiffness(100);
			soft.parametrize(CartDOF.TRANSL).setStiffness(600);
				
		handle = lbr.moveAsync(positionHold(soft, -1, TimeUnit.SECONDS));
		sel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,
			"Collision Dection and LBR act gentle",
			"Move Continue",//0
			"return");//1
		handle.cancel();
		if (sel != 0) {
			resumeMotion = false;
			lbr.move(ptp(getFrame("/start")).setJointVelocityRel(.3));
		}
				
		return resumeMotion;
	}	
	
	
	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		FeatureDemo app = new FeatureDemo();
		app.runApplication();
	}

}

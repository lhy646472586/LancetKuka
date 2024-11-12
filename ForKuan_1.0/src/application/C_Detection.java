package application;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.Date;

import javax.inject.Inject;

import sun.security.action.GetLongAction;

//import applications.AppMainMenu;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.ConditionObserver;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.IRisingEdgeListener;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;
import com.kuka.roboticsAPI.conditionModel.NotificationType;
import com.kuka.roboticsAPI.conditionModel.ObserverManager;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.LBR;

public class C_Detection {
	
	
	@Inject
	private LBR robot;
	private double _torquelimit = 5; // N*m
	private double pro;
	private double _jointTorque1, _jointTorque2, _jointTorque3, _jointTorque4,
			_jointTorque5, _jointTorque6, _jointTorque7;
	private ICondition joint1, joint2, joint3, joint4, joint5, joint6, joint7;
	private ICondition conditionTorque;
	private boolean bstop =false;
	private double[] torqueValue = new double[7];

	private ConditionObserver torqueObserver;
	private ObserverManager ObserverM;
	private RoboticsAPIApplication _app;

	public C_Detection(LBR robot,RoboticsAPIApplication app,ObserverManager obsM) {
		this.ObserverM = obsM;
		this.robot = robot;
		this._app = app; 
		initialize();
	};

	private void initialize() {
		
		_jointTorque1 = robot.getExternalTorque().getSingleTorqueValue(
				JointEnum.J1);
		_jointTorque2 = robot.getExternalTorque().getSingleTorqueValue(
				JointEnum.J2);
		_jointTorque3 = robot.getExternalTorque().getSingleTorqueValue(
				JointEnum.J3);
		_jointTorque4 = robot.getExternalTorque().getSingleTorqueValue(
				JointEnum.J4);
		_jointTorque5 = robot.getExternalTorque().getSingleTorqueValue(
				JointEnum.J5);
		_jointTorque6 = robot.getExternalTorque().getSingleTorqueValue(
				JointEnum.J6);
		_jointTorque7 = robot.getExternalTorque().getSingleTorqueValue(
				JointEnum.J7);
		torqueValue[0] = _jointTorque1;
		torqueValue[1] = _jointTorque2;
		torqueValue[2] = _jointTorque3;
		torqueValue[3] = _jointTorque4;
		torqueValue[4] = _jointTorque5;
		torqueValue[5] = _jointTorque6;
		torqueValue[6] = _jointTorque7;
//				,_jointTorque2,_jointTorque3,_jointTorque4,_jointTorque5,_jointTorque6,_jointTorque7};

		joint1 = new JointTorqueCondition(robot, JointEnum.J1, -_torquelimit
				+ _jointTorque1, _torquelimit + _jointTorque1);
		joint2 = new JointTorqueCondition(robot, JointEnum.J1, -_torquelimit
				+ _jointTorque2, _torquelimit + _jointTorque2);
		joint3 = new JointTorqueCondition(robot, JointEnum.J1, -_torquelimit
				+ _jointTorque3, _torquelimit + _jointTorque3);
		joint4 = new JointTorqueCondition(robot, JointEnum.J1, -_torquelimit
				+ _jointTorque4, _torquelimit + _jointTorque4);
		joint5 = new JointTorqueCondition(robot, JointEnum.J1, -_torquelimit
				+ _jointTorque5, _torquelimit + _jointTorque5);
		joint6 = new JointTorqueCondition(robot, JointEnum.J1, -_torquelimit
				+ _jointTorque6, _torquelimit + _jointTorque6);
		joint7 = new JointTorqueCondition(robot, JointEnum.J1, -_torquelimit
				+ _jointTorque7, _torquelimit + _jointTorque7);

		conditionTorque = joint1.or(joint2, joint3, joint4, joint5, joint6,
				joint7);
		bstop =false;
	};

	    IRisingEdgeListener mypause = new IRisingEdgeListener() {

		@Override
		public void onRisingEdge(ConditionObserver conditionTorque,
				Date time, int missedEvents) {
		
			
			System.out.println(""+torqueObserver.getCondition());
			System.out.println("Listener");
			// TODO Auto-generated method stub
			
			System.out.println("Override Current"+pro);
			System.out.println("Torque Current"+Arrays.toString(torqueValue));
		 
			
			if (!bstop) {
				 pro = _app.getApplicationControl().getApplicationOverride();
				_app.getApplicationControl().setApplicationOverride(0);
				
				ThreadUtil.milliSleep(1000);
				bstop = true ; 
			}else if(_app.getApplicationControl().getApplicationOverride()==0 && bstop){
				_app.getApplicationControl().setApplicationOverride(pro);
				bstop = false;
				ThreadUtil.milliSleep(1000);
			}
			
			System.out.println("End Override Current "+pro);
		
			

		}

	};

	public void cd_enable() {
		torqueObserver = ObserverM.createConditionObserver(conditionTorque,NotificationType.OnEnable, mypause);
		torqueObserver.enable();
		
		
	}

	public void cd_disable() {
		
		torqueObserver.disable();
	}

}

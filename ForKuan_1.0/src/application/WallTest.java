package application;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServoLIN.ISmartServoLINRuntime;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.generated.ioAccess.SafeDataIOGroup;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.task.ITaskLogger;

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
public class WallTest extends RoboticsAPIApplication {
	@Inject
	private LBR lbr;

	@Inject
	private ITaskLogger log;

//	@Named("Tool_2")
//	@Inject
//	private Tool tool;
	
	
	@Named("Tool_2")
	
	@Inject
	private Tool needle;
	
	
	private Controller kuka_Sunrise_Cabinet_1;
	private double fx;
	private double px;
	private double dfx;
	private double fy;
	private double py;
	private double dfy;
	private double fz;
	private double pz;
	private double dfz;
	private double fa;
	private double pa;
	private double dfa;
	private double fb;
	private double pb;
	private double dfb;
	private double fc;
	private double pc;
	private double dfc;
	public static int nWorkingmode=0;
	
	

	private double cx;
	private double cy;
	private double cz;
	private double ca;
	private double cb;
	private double cc;
	

	private double ccx;
	private double ccy;
	private double ccz;
	private double cca;
	private double ccb;
	private double ccc;

	public double jointLimit_1;
	public double jointLimit_2;
	public double jointLimit_3;
	public double jointLimit_4;
	public double jointLimit_5;
	public double jointLimit_6;
	public double jointLimit_7;
    private Tool _toolAttachedToLBR;
    private LoadData _loadData1;
    private JointPosition jointPos;
    // Tool Data
    private static final String TOOL_FRAME1 = "toolFrame";
    private static final double[] TRANSLATION_OF_TOOL1 = { 0, 0, 0 };
    private static final double MASS1 = 2;
    private static final double[] CENTER_OF_MASS_IN_MILLIMETER1 = { 0, 0, 0 };
    private SafeDataIOGroup SafeDataIO;
	
	private CartesianImpedanceControlMode cic;
	public CartesianImpedanceControlMode cic1;
	boolean bstop = false;

	@Override
	public void initialize() {
		// initialize your application here
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		SafeDataIO = new SafeDataIOGroup(kuka_Sunrise_Cabinet_1);
		_loadData1 = new LoadData();
        _loadData1.setMass(0);
        _loadData1.setCenterOfMass(
                CENTER_OF_MASS_IN_MILLIMETER1[0], CENTER_OF_MASS_IN_MILLIMETER1[1],
                CENTER_OF_MASS_IN_MILLIMETER1[2]);
        _toolAttachedToLBR = new Tool("Tool", _loadData1);

        XyzAbcTransformation trans = XyzAbcTransformation.ofTranslation(
                TRANSLATION_OF_TOOL1[0], TRANSLATION_OF_TOOL1[1],
                TRANSLATION_OF_TOOL1[2]);
        ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME1
                + "(TCP)", trans);
        _toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
        // Attach tool to the robot
        _toolAttachedToLBR.attachTo(lbr.getFlange());
		
        cic = new CartesianImpedanceControlMode();
        cic.parametrize(CartDOF.TRANSL).setStiffness(2000);
        needle.attachTo(lbr.getFlange());
	}

	public HandGuidingMotion createhandGuidingMotion(){
		
		HandGuidingMotion motion = new HandGuidingMotion();
		motion.setJointVelocityLimit(0.8)
		.setCartVelocityLimit(900.0).setJointLimitViolationFreezesAll(false)
		.setJointLimitsMax(Math.toRadians(30), Math.toRadians(0), Math.toRadians(45), Math.toRadians(120), +0.087,+1.571, +0.087)
		.setJointLimitsMin(Math.toRadians(-30), Math.toRadians(-75), Math.toRadians(-25), Math.toRadians(0), -0.087,-1.571, -0.087)
		.setJointLimitsEnabled(true,true,true,true,false,false,false)


;
		return motion;
	}
	
	
	@Override
	public void run() {	
	
		cic.parametrize(CartDOF.X).setStiffness(5000);
		while (true)
		{
			
			if (SafeDataIO.getInput4()==true){
					//ThreadUtil.milliSleep(500);
					if(SafeDataIO.getInput4()==true)
					{
						System.out.println("handguiding mode.");
						//其他指的是打磨模式
						needle.getFrame("/tcp_3").move(createhandGuidingMotion());
					}

					nWorkingmode=0;
			}
			else{
				   if (!ServoMotion.validateForImpedanceMode(lbr))
			        {
			            getLogger()
			                    .info("Validation of torque model failed - correct your mass property settings");
			            getLogger()
			                    .info("Servo motion will be available for position controlled mode only, until validation is performed");
			        }

//				lbr.move(ptp(getFrame("/P2")).setJointVelocityRel(0.5));

				Frame current = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_3"));
				SmartServoLIN aSmartServoLINMotion = new SmartServoLIN(current);
				aSmartServoLINMotion.setMinimumTrajectoryExecutionTime(20e-3);

				log.info("Start Smart Servo Lin motion");

				 _toolAttachedToLBR.moveAsync(aSmartServoLINMotion);

				getLogger().info("Get the runtime of the SmartServoLIN motion");
				ISmartServoLINRuntime smartServoLINRuntime = aSmartServoLINMotion
						.getRuntime();

				px = py = pz = 0.05;
				pa = pb = pc = 0.0005;

				Frame destFrame = current.copyWithRedundancy();
				Frame destFrame_Dangerous = current.copyWithRedundancy();
				
				double zFx = lbr.getExternalForceTorque(needle.getFrame("/tcp_2")).getForce()
						.getX();
				double zFy = lbr.getExternalForceTorque(needle.getFrame("/tcp_2")).getForce()
						.getY();
				double zFz = lbr.getExternalForceTorque(needle.getFrame("/tcp_2")).getForce()
						.getZ();
				double zFa = lbr.getExternalForceTorque(needle.getFrame("/tcp_2")).getTorque().getZ();
				double zFb = lbr.getExternalForceTorque(needle.getFrame("/tcp_2")).getTorque().getY();
				double zFc = lbr.getExternalForceTorque(needle.getFrame("/tcp_2")).getForce().getX();

				ccx = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2")).getX();
				ccy = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2")).getY();
				ccz = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2")).getZ();
				cca = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2")).getAlphaRad();
				ccb = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2")).getBetaRad();
				ccc = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2")).getGammaRad();
				
				
				
				log.info("get Force in x " + zFx);
				log.info("get Force in y " + zFy);
				log.info("get Force in z " + zFz);
				log.info("get Torque in a " + cca);
				log.info("get Torque in b " + ccb);
				log.info("get Torque in c " + ccc);

				try {
					
					while (SafeDataIO.getInput4()==false) {
				        cic1 = new CartesianImpedanceControlMode();
				        cic1.parametrize(CartDOF.ROT).setStiffness(300);
				        cic1.parametrize(CartDOF.X).setStiffness(5000);
				        cic1.parametrize(CartDOF.Y).setStiffness(5000);
				        cic1.parametrize(CartDOF.Z).setStiffness(50);
				        
						fx = lbr.getExternalForceTorque(needle.getFrame("/tcp_2")).getForce()
								.getX();
						fy = lbr.getExternalForceTorque(needle.getFrame("/tcp_2")).getForce()
								.getY();
						fz = lbr.getExternalForceTorque(needle.getFrame("/tcp_2")).getForce()
								.getZ();
						fa = lbr.getExternalForceTorque(needle.getFrame("/tcp_2")).getTorque().getZ();
						fb = lbr.getExternalForceTorque(needle.getFrame("/tcp_2")).getTorque().getY();
						fc = lbr.getExternalForceTorque(needle.getFrame("/tcp_2")).getTorque().getX();
						
						dfx = 1 * px * (fx - zFx);
						dfy = 1 * py * (fy - zFy);
						dfz = 1 * pz * (fz - zFz);
						dfa = 1 * pa * (fa - zFa);
						dfb = 1 * pb * (fb - zFb);
						dfc = 1 * pc * (fc - zFc);

						cx = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2")).getX();
						cy = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2")).getY();
						cz = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2")).getZ();
						ca = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2")).getAlphaRad();
						cb = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2")).getBetaRad();
						cc = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2")).getGammaRad();
						
						if (Math.abs(dfx) < 0.1) {
							dfx = 0;
						}
						
						if (ccx - cx > 50) {
							if (fx > 0) {
//								dfx = 0;

							}
						}

						if (ccx - cx < -50) {
							if (fx < 0) {
//								dfx = 0;

							}
						}

						if (Math.abs(dfy) < 0.1) {
							dfy = 0;
						}
						if (ccy - cy >= 50) {
							if (fy < 0) {
//								dfy = 0;

							}
						}
						if (ccy - cy <= -50) {
							if (fy > 0) {
//								dfy = 0;

							}
						}

						
						
						
						if (Math.abs(dfz) < 0.1) {
							dfz = 0;
						}
						if (ccz - cz > 5) {
							if (fz > 0) {
//								dfz = 0;

							}
						}

						if (ccz - cz < -5) {
							if (fz < 0) {
//								dfz = 0;

							}

						}

//						dfz = 0;
						// log.info("get Force in delta x" + dfx + ",y " + dfy + ",z " +
						// dfz);
						
						if(dfx>0.6){
							dfx=0.6;
						}
						if(dfx<-0.6){
							dfx=-0.6;
						}
						
						if(dfy>0.6){
							dfy=0.6;
						}
						if(dfy<-0.6){
							dfy=-0.6;
						}
						
						if(dfz>0.6){
							dfz=0.6;
						}
						if(dfz<-0.6){
							dfz=-0.6;
						}
						if(dfa>0.1){
							dfa=0.1;
						}
						if(dfa<-0.1){
							dfa=-0.1;
						}
						
//						Frame Ptest2 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy().transform((Transformation.ofTranslation(0, 200, 0)));
//						Frame Ptest2 = destFrame.transform((Transformation.ofTranslation(dfx, dfy, 0)));
						Frame Ptest3 = destFrame_Dangerous.transform((Transformation.ofTranslation(dfx*0.1, dfy*0.1, 0)));

//						Frame Ptest2 = destFrame.transform((Transformation.ofRad(dfx, dfy, dfz, dfa, dfb, dfc)));
						Frame Ptest2 = destFrame.transform((Transformation.ofRad(dfx*2, 0, 0, 0, 0, 0)));
//						Frame Ptest2 = destFrame.transform((Transformation.ofRad(0, 0, 0, 0, 0, 0)));
//						destFrame.setX(destFrame.getX() + dfx);
//						destFrame.setY(destFrame.getY() + dfy);
//						destFrame.setZ(destFrame.getZ() + dfz);
						
						
						destFrame.setX(Ptest2.getX());
						destFrame.setY(Ptest2.getY());
						destFrame.setZ(Ptest2.getZ());
						destFrame.setAlphaRad(Ptest2.getAlphaRad());
						destFrame.setBetaRad(Ptest2.getBetaRad());
						destFrame.setGammaRad(Ptest2.getGammaRad());
						
						destFrame_Dangerous.setX(Ptest3.getX());
						destFrame_Dangerous.setY(Ptest3.getY());
						destFrame_Dangerous.setZ(Ptest3.getZ());
						
						
//						Frame Ptest1= current.copyWithRedundancy();
//						Frame Ptest2 = Ptest1.transform((Transformation.ofTranslation(dfx, dfy, dfz)));
						smartServoLINRuntime.updateWithRealtimeSystem();
						
						jointPos=lbr.getInverseKinematicFromFrameAndRedundancy(destFrame);
////						System.out.println("J1:"+Math.toDegrees(jointPos.get(JointEnum.J1))+" J2:"+Math.toDegrees(jointPos.get(JointEnum.J2))+" J3:"+Math.toDegrees(jointPos.get(JointEnum.J3))+" J4:"+Math.toDegrees(jointPos.get(JointEnum.J4))+" J5:"+Math.toDegrees(jointPos.get(JointEnum.J5))+" J6:"+Math.toDegrees(jointPos.get(JointEnum.J6))+" J7:"+Math.toDegrees(jointPos.get(JointEnum.J7)));
						jointLimit_1=Math.toDegrees(jointPos.get(JointEnum.J1));
						jointLimit_2=Math.toDegrees(jointPos.get(JointEnum.J2));
						jointLimit_3=Math.toDegrees(jointPos.get(JointEnum.J3));
						jointLimit_4=Math.toDegrees(jointPos.get(JointEnum.J4));
						jointLimit_5=Math.toDegrees(jointPos.get(JointEnum.J5));
						jointLimit_6=Math.toDegrees(jointPos.get(JointEnum.J6));
						jointLimit_7=Math.toDegrees(jointPos.get(JointEnum.J7));
						
						//判断是否能到达限位
						if(Math.abs(jointLimit_1)<160 && Math.abs(jointLimit_2)<110 && Math.abs(jointLimit_3)<160 && Math.abs(jointLimit_4)<110 && Math.abs(jointLimit_5)<160 && Math.abs(jointLimit_6)<110 && Math.abs(jointLimit_7)<165){
							px = py = pz = 0.05;
						}
						else{
							double nMin=Math.min(Math.abs(170-Math.abs(jointLimit_1)), 120-Math.abs(Math.abs(jointLimit_2)));
							nMin=Math.min(nMin, Math.abs(170-Math.abs(jointLimit_3)));
							nMin=Math.min(nMin, Math.abs(120-Math.abs(jointLimit_4)));
							nMin=Math.min(nMin, Math.abs(170-Math.abs(jointLimit_5)));
							nMin=Math.min(nMin, Math.abs(120-Math.abs(jointLimit_6)));
							nMin=Math.min(nMin, Math.abs(175-Math.abs(jointLimit_7)));
							px = py = pz = 0.05*(Math.pow(nMin, 3)/2000);
						} 
//						double[] aa=  {20,20,20};
//						smartServoLINRuntime.setMaxTranslationVelocity(aa);
						smartServoLINRuntime.setDestination(destFrame);
						
						
//						smartServoLINRuntime.setDestination(Ptest2);

//						smartServoLINRuntime.changeControlModeSettings(cic);
						
						if (lbr.getCurrentCartesianPosition(lbr.getFlange()).getX() > 700) {
//							bstop = true;
						}

						//ThreadUtil.milliSleep(10);
					}

				} catch (Exception e) {
					// TODO: handle exception
					log.info(e.toString());
				}

				smartServoLINRuntime.stopMotion();
			}
			
			//ThreadUtil.milliSleep(50);
		}
		
	}

	@Override
	public void dispose() {
		// TODO Auto-generated method stub
		bstop = true;
		super.dispose();
	}
}
 package application;



import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.math.BigDecimal;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.common.StatisticTimer;
import com.kuka.common.ThreadUtil;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.connectivity.motionModel.directServo.DirectServo;
import com.kuka.connectivity.motionModel.directServo.IDirectServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.connectivity.motionModel.smartServoLIN.ISmartServoLINRuntime;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.generated.ioAccess.MytestIOIOGroup;
import com.kuka.generated.ioAccess.SafeDataIOGroup;
import com.kuka.med.unmasteredApp.MedApplicationCategory;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPITask;
import com.kuka.roboticsAPI.applicationModel.tasks.UseRoboticsAPIContext;
import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.sunrise.ISafetyState;
import com.kuka.roboticsAPI.deviceModel.CartesianVelocityLimitInfo;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.ExecutionState;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.ITransformation;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Rotation;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import static com.kuka.roboticsAPI.motionModel.HRCMotions.*;

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
@MedApplicationCategory(checkMastering = false)
public class TCPServerSendDataApplication extends RoboticsAPIApplication {
//    int i=0;
	private HandGuidingMotion motion;


	@Inject
	private  LBR lbr;
	private Tool _toolAttachedToLBR;
	private Controller kuka_Sunrise_Cabinet_1;
    @Inject
    private MytestIOIOGroup io;
    private static final int AIM = 10;
	private static int i = 0;
	private CartesianImpedanceControlMode _nullspacemode ;
	IMotion bHandGuidingStart;
	ServerSocket serverSocket=null;
	ServerSocket serverSocketSend=null;
	DataOutputStream outputStream=null;
	OutputStreamWriter writer=null;
	OutputStreamWriter writer_recive=null;
	Socket socket=null;
	Socket socket_recive=null;
	DataOutputStream outputStream_recive=null;
	 private static JointPosition HOME = new JointPosition(0, Math.toRadians(8), 0, 
    		Math.toRadians(75), 0, Math.toRadians(-80), 0);
	 private static final double FREQENCY = 0.1;
	 private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 30;
	 private LoadData _loadData;
	 private static final double[] TRANSLATION_OF_TOOL = { -150.7, 0, 227.9 };
	 private static final double MASS = 0;
	 private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 35.3, 0, 101.3 };
	 private static final String TOOL_FRAME = "toolFrame";
     private static final double[] MAX_TRANSLATION_VELOCITY = { 150, 150, 150 };
     private ISmartServoLINRuntime _smartServoLINRuntime = null;
	 private static final int NUM_RUNS = 600;
	 private static final double AMPLITUDE = 70;
	 private SafeDataIOGroup SafeDataIO;
	 private JointPosition jointPos;
	 private JointPosition jointPos_zuo;
	 private JointPosition jointPos_you;
	 Frame pre_Place;
//	@Named("gripper")
//	@Inject
//	private Tool gripper;
	
//	@Named("Tool_2")
//	@Inject
//	private Tool Tool_2;
	
	@Named("Tool_2")
	
	@Inject
	private Tool needle;
	
	@Named("gripper")
	@Inject
	private Tool needle_gripper;
	@Inject
	private BrakeTestMonitorSampleApp BreakTest;
	private ObjectFrame tcp;
	private ObjectFrame tcp_2;
	private ObjectFrame handle;
	private BooleanIOCondition VaccumDetect;
	private Frame startpos;
	private Tool tool;
	public boolean bDangerous=false;
	public boolean breaktest=false;
	
	//å�‘é€�å­—ç¬¦ä¸² data0~data18
	public static String data0="0";
	public static String data1="0";
	public static String data2="0";
	public static String data3="0";
	public static String data4="0";
	public static String data5="0";
	public static String data6="0";
	public static String data7="0";
	public static String data8="0";
	public static String data9="0";
	public static String data10="0";
	public static String data11="0";
	public static String data12="0";
	public static String data13="0";
	public static String data14="0";
	public static String data15="0";
	public static String data16="0";
	public static String data17="0";
	public static String data18="0";
    
	//å…¨å±€X,Y,Zå�˜é‡� è¾“å…¥å�˜é‡�
	public static double nX=0;
	public static double nY=0;
	public static double nZ=0;
	public static double nA=0;
	public static double nB=0;
	public static double nC=0;
	public String Err="0,";
	public double nTest_rotation=0;
	 JointPosition currentPos_test;

	//å…¨å±€å·¥ä½œæ¨¡å¼�å�˜é‡� è¾“å…¥å�˜é‡�
	public static int nWorkingmode=0;
	public static int nToolMode=0;
	
	private JointPosition jointPos_register;
	@Inject
	private CopyOfTeachingByHand_2 JointImpedanceMode;
	@Override
	public void initialize() {
		nWorkingmode=0;
		nToolMode=1;
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		SafeDataIO = new SafeDataIOGroup(kuka_Sunrise_Cabinet_1);
		nX=0;
		nY=0;
		nZ=0;
		nA=0;
		nB=0;
		nC=0;
		Err="0,";
		io.setOutput5(false);
		jointPos=new JointPosition(   Math.toRadians(9.44),
                Math.toRadians(5.15),
                Math.toRadians(-17.39),
                Math.toRadians(61.02),
                Math.toRadians(29.90),
                Math.toRadians(-99.06),
                Math.toRadians(-9.62));
		
		jointPos_zuo=new JointPosition(   Math.toRadians(6.95),
                Math.toRadians(-7.41),
                Math.toRadians(40.42),
                Math.toRadians(115),
                Math.toRadians(-2.17),
                Math.toRadians(-55.7),
                Math.toRadians(122));
		
		jointPos_you=new JointPosition(   Math.toRadians(-9.91),
                Math.toRadians(-4.63),
                Math.toRadians(-44.98),
                Math.toRadians(115),
                Math.toRadians(-2.29),
                Math.toRadians(-65.18),
                Math.toRadians(-127));
		

		
		
		try {
			ThreadUtil.milliSleep(1000);
			if(serverSocket!=null){
				serverSocket.close();
				serverSocket=null;
				System.out.println("022");
			}
			if(serverSocketSend!=null ){
				serverSocketSend.close();
				serverSocketSend=null;
				System.out.println("02222");
			}
			if(writer!=null ){
				writer.close();
				writer=null;
				System.out.println("0333");
			}
			if(outputStream!=null ){
				outputStream.close();
				outputStream=null;
				System.out.println("033333");
			}
			if(socket!=null ){
				socket.close();
				socket=null;
				System.out.println("033333");
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		
       lbr = getContext().getDeviceFromType(LBR.class);

     	_loadData = new LoadData();
        _loadData.setMass(MASS);
        _loadData.setCenterOfMass(
                CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
                CENTER_OF_MASS_IN_MILLIMETER[2]);
        _toolAttachedToLBR = new Tool("Tool", _loadData);

        XyzAbcTransformation trans = XyzAbcTransformation.ofRad(TRANSLATION_OF_TOOL[0], TRANSLATION_OF_TOOL[1], TRANSLATION_OF_TOOL[2],0,-1.047,0);
        ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME
                + "(TCP)", trans);
        _toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
        // Attach tool to the robot
        _toolAttachedToLBR.attachTo(lbr.getFlange());
        
		needle.attachTo(lbr.getFlange());
		needle_gripper.attachTo(lbr.getFlange());
	}

	public  class sendRTdata implements Callable<String> {
         
		 public void GetData()
		    {
			// ThreadUtil.milliSleep(1000);
				data0 = "$0,";
				data1 = "0,";
				data2 = Err;
				data3 = "1,";
				double a1 =getApplicationControl().getApplicationOverride();
				BigDecimal bigDecimal0 = new BigDecimal(a1);
				a1 = bigDecimal0.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data4 = String.valueOf(a1)+",";
				data5 = "0,";
				JointPosition actPos = lbr.getCurrentJointPosition();
				a1 = Math.toDegrees(actPos.get(0));
				BigDecimal bigDecimal = new BigDecimal(a1);
				a1 = bigDecimal.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data6=String.valueOf(a1)+",";
				a1 = Math.toDegrees(actPos.get(1));
				BigDecimal bigDecimal1 = new BigDecimal(a1);
				a1 = bigDecimal1.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data7=String.valueOf(a1)+",";
				a1 = Math.toDegrees(actPos.get(2));
				BigDecimal bigDecimal2 = new BigDecimal(a1);
				a1 = bigDecimal2.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data8=String.valueOf(a1)+",";
				a1 = Math.toDegrees(actPos.get(3));
				BigDecimal bigDecimal3 = new BigDecimal(a1);
				a1 = bigDecimal3.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data9=String.valueOf(a1)+",";
				a1 = Math.toDegrees(actPos.get(4));
				BigDecimal bigDecimal4 = new BigDecimal(a1);
				a1 = bigDecimal4.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data10=String.valueOf(a1)+",";
				a1 = Math.toDegrees(actPos.get(5));
				BigDecimal bigDecimal5 = new BigDecimal(a1);
				a1 = bigDecimal5.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data11=String.valueOf(a1)+",";
				a1 = Math.toDegrees(actPos.get(6));
				BigDecimal bigDecimal6 = new BigDecimal(a1);
				a1 = bigDecimal6.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data12=String.valueOf(a1)+",";
				Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"));
				
				
				
				
				if (nToolMode==1){
						cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiPolish"));
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiPolish"), cmdPos2);
				}
				else if(nToolMode==2)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanPolish"), cmdPos2);
				}	
				else if(nToolMode==3)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiSet"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiSet"), cmdPos2);
				}	
				else if(nToolMode==4)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanSet"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanSet"), cmdPos2);		
				}	
				else if(nToolMode==5)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiPolish"), cmdPos2);	
				}	
				else if(nToolMode==6)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanPolish"), cmdPos2);	
				}	
				else if(nToolMode==7)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiSet"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiSet"), cmdPos2);
				}	
				else if(nToolMode==8)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanSet"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanSet"), cmdPos2);
				}	
		  else if (nToolMode==9){
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"), cmdPos2);
			}
			else if(nToolMode==10)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanPolish"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanPolish"), cmdPos2);
			}	
			else if(nToolMode==11)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiSet"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiSet"), cmdPos2);
			}	
			else if(nToolMode==12)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanSet"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanSet"), cmdPos2);		
			}	
			else if(nToolMode==13)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiPolish"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiPolish"), cmdPos2);	
			}	
			else if(nToolMode==14)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanPolish"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanPolish"), cmdPos2);	
			}	
			else if(nToolMode==15)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiSet"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiSet"), cmdPos2);
			}	
			else if(nToolMode==16)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanSet"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanSet"), cmdPos2);
			}	
			  else if (nToolMode==17){
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiPolish"), cmdPos2);
			}
			else if(nToolMode==18)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanPolish"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanPolish"), cmdPos2);
			}	
			else if(nToolMode==19)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiSet"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiSet"), cmdPos2);
			}	
			else if(nToolMode==20)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanSet"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanSet"), cmdPos2);		
			}	
			else if(nToolMode==21)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiPolish"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiPolish"), cmdPos2);	
			}	
			else if(nToolMode==22)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanPolish"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanPolish"), cmdPos2);	
			}	
			else if(nToolMode==23)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiSet"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiSet"), cmdPos2);
			}	
			else if(nToolMode==24)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanSet"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanSet"), cmdPos2);
			}
			  else if (nToolMode==25){
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiPolish"), cmdPos2);
			}
			else if(nToolMode==26)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanPolish"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanPolish"), cmdPos2);
			}	
			else if(nToolMode==27)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiSet"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiSet"), cmdPos2);
			}	
			else if(nToolMode==28)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanSet"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanSet"), cmdPos2);		
			}	
			else if(nToolMode==29)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiPolish"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiPolish"), cmdPos2);	
			}	
			else if(nToolMode==30)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanPolish"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanPolish"), cmdPos2);	
			}	
			else if(nToolMode==31)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiSet"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiSet"), cmdPos2);
			}	
			else if(nToolMode==32)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanSet"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanSet"), cmdPos2);
			}
			  else if (nToolMode==33){
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiPolish"), cmdPos2);
			}
			else if(nToolMode==34)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanPolish"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanPolish"), cmdPos2);
			}	
			else if(nToolMode==35)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiSet"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiSet"), cmdPos2);
			}	
			else if(nToolMode==36)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanSet"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanSet"), cmdPos2);		
			}	
			else if(nToolMode==37)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiPolish"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiPolish"), cmdPos2);	
			}	
			else if(nToolMode==38)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanPolish"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanPolish"), cmdPos2);	
			}	
			else if(nToolMode==39)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiSet"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiSet"), cmdPos2);
			}	
			else if(nToolMode==40)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanSet"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanSet"), cmdPos2);
			}
				else
				{
					 cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"));
						cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"));
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"), cmdPos2);
				}
				
				a1=cmdPos.getX();
				BigDecimal bigDecimal7 = new BigDecimal(a1);
				a1 = bigDecimal7.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//				data13=a1+",";
				data13=String.valueOf(a1)+",";
				
				//è½´å��æ ‡y
				a1=cmdPos.getY();
				BigDecimal bigDecimal8 = new BigDecimal(a1);
				a1 = bigDecimal8.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//				data14=a1+",";
				data14=String.valueOf(a1)+",";
				
				//è½´å��æ ‡z
				a1=cmdPos.getZ();
				BigDecimal bigDecimal9 = new BigDecimal(a1);
				a1 = bigDecimal9.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//				data15=a1+",";
				data15=String.valueOf(a1)+",";
				
				//è½´å��æ ‡rx
				a1=Math.toDegrees(cmdPos.getAlphaRad());
				BigDecimal bigDecima20 = new BigDecimal(a1);
				a1 = bigDecima20.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//				data16=a1+",";
				data16=String.valueOf(a1)+",";
				
				//è½´å��æ ‡ry
				a1=Math.toDegrees(cmdPos.getBetaRad());
				BigDecimal bigDecima21 = new BigDecimal(a1);
				a1 = bigDecima21.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//				data17=a1+",";
				data17=String.valueOf(a1)+",";
				
				//è½´å��æ ‡rz

				
				a1=Math.toDegrees(cmdPos.getGammaRad());
				BigDecimal bigDecima22 = new BigDecimal(a1);
				a1 = bigDecima22.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//				data18=a1+"$";
				data18=String.valueOf(a1)+",$";
		    }
		
		public String call() {
			
			
			
			
			while(true){
			try {
			boolean bPause=false;
			serverSocketSend = new ServerSocket(30001);
			
			
			
			System.out.println("New socket.");
			socket= serverSocketSend.accept();
//			socket.setSoTimeout(2500);
			System.out.println("Socket accepted. IP:{" + socket.getInetAddress().getHostAddress() + "}.");
			outputStream = new DataOutputStream(socket.getOutputStream());
			writer= new OutputStreamWriter(outputStream);

			while (bPause==false)
			{


				try{
					GetData();
//					ThreadUtil.milliSleep(1000);
//					System.out.println("x1");
////				    System.out.println(socket_recive.isBound());
////				    System.out.println(socket_recive.isConnected());
//				    System.out.println("x2");
////				    System.out.println(socket_recive.isClosed());
//				    System.out.println("x3");
//				    System.out.println(socket_recive.getOOBInline());
				    
					String data=data0+data1+data2+data3+data4+data5+data6+data7+data8+data9+data10+data11+data12+data13+data14+data15+data16+data17+data18;

			
					writer.write(data);
					writer.flush();
					ThreadUtil.milliSleep(50);
//					ThreadUtil.milliSleep(2000);
					}
				catch (IOException e) {
					System.out.println("Socket closed.");					
					System.out.println("Closing socket.");
					
					writer.close();
					outputStream.close();
					socket.close();
					serverSocketSend.close();
					writer=null;
					outputStream=null;
					socket=null;
					serverSocketSend=null;
					bPause=true;
					
				}
				

			}
			
			System.out.println("closed.");					
			System.out.println("Socket.");
			

		} catch (IOException e) {
			e.printStackTrace();
		};
		}
		}
	}

	
	
	//////111111111
public  class reciveRTdata implements Callable<String> {
        
        
		
		public String call() {	
			String[] units=null;
			while(true){
			try{
			boolean bPause=false;
			serverSocket = new ServerSocket(30007);
			
			System.out.println("New socket.");
		    socket_recive = serverSocket.accept();
//		    socket_recive.setSoTimeout(2500);
			System.out.println("Socket accepted. IP:{" + socket_recive.getInetAddress().getHostAddress() + "}.");
		    
			InputStream in = socket_recive.getInputStream();
			outputStream_recive = new DataOutputStream(socket_recive.getOutputStream());
			writer_recive= new OutputStreamWriter(outputStream_recive);	
			while(bPause==false){

				try{
					

					
				      byte[] buf = new byte[1];
				        int size = 0;
//				        System.out.println("qq");
				        StringBuffer sb = new StringBuffer();
				    
//				        System.out.println(in.read(buf,0,buf.length));
				        if (in.read(buf,0,buf.length)==-1)
				        {
				        	bPause=true;
//				        	System.out.println("oo");
				        	break;
				        }
//				        System.out.println("ee");
				        while (( size = in.read(buf,0,buf.length)) != -1) {
//				            System.out.println("ww");
				            String str = new String(buf);

				            sb.append(str);
//				            System.out.print(str);
//				            System.out.println(str);
				            if(str.equals("*")) {
//								System.out.println(new String(sb).trim());
//								System.out.println("tt");
								units = new String(sb).trim().split(",");
								
				                break;
				            }
				        }
//				        System.out.println("rr");
				        

					if(units[1] == null){
						System.out.println("data receiving error: no command received.....");
					}else{
						if(units[1].equals("spd")){
//						
							String para = units[2].substring(0, units[2].length() - 1);
//							System.out.println("spd: " + para);
							writer_recive.write("$res,spd,1");
							writer_recive.flush();
							
						}
						else if(units[1].equals("mp")){
//							System.out.println("para: " + units[2]);
//							System.out.println("para: " + units[3]);
//							System.out.println("para: " + units[4]);
//							System.out.println("para: " + units[5]);
//							System.out.println("para: " + units[6]);
							String para1 = units[7].substring(0, units[7].length() - 1);
//							System.out.println("para" + para1);

							nX=Double.parseDouble(units[2]);
							nY=Double.parseDouble(units[3]);
							nZ=Double.parseDouble(units[4]);
							nA=Double.parseDouble(units[5]);
							nB=Double.parseDouble(units[6]);
							nC=Double.parseDouble(para1);
							
							//ss

							
							pre_Place = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
							pre_Place.setX(nX);
							pre_Place.setY(nY);
							pre_Place.setZ(nZ);
							pre_Place.setAlphaRad(Math.toRadians(nA));
							pre_Place.setBetaRad(Math.toRadians(nB));
							pre_Place.setGammaRad(Math.toRadians(nC));
//							System.out.println("pre_Place"+pre_Place);
//							System.out.println("nX"+nX+"  nY"+nY+"  nZ"+nZ+"  nA"+nA+"  nB"+nB+"  nC"+nC);
							writer_recive.write("$para,mp,0$");
							writer_recive.flush();
						}
						else if(units[1].equals("sIO")){
//							System.out.println("para: " + units[2]);
							String para2 = units[3].substring(0, units[3].length() - 1);
							if(Double.parseDouble(para2)==1 && Double.parseDouble(units[2])==1)
							{
								io.setOutput5(true);	
								//System.out.println("io.setOutput5(true)");
							}
							else if(Double.parseDouble(para2)==0 && Double.parseDouble(units[2])==1)
							{
								io.setOutput5(false);
								//System.out.println("io.setOutput5(false)");
								//System.out.println("*"+Double.parseDouble(para2)+"*  "+"*"+Double.parseDouble(units[2])+"*");
							}
							else if(Double.parseDouble(para2)==8 && Double.parseDouble(units[2])==1)
							{
								breaktest=true;	
								System.out.println("breaktest=true;");
							}
//							System.out.println("sIO: " + para2);
							writer_recive.write("$res,sIO,0$");
							writer_recive.flush();
							
						}
						else if(units[1].equals("RobotMove")){
							System.out.println("RobotMove " + units);
//							System.out.println("para: " + units[2]);
							String para2 = units[3].substring(0, units[3].length() - 1);
							if(Double.parseDouble(para2)==2 )
							{
								System.out.println("RobotMove2");
								nToolMode=2;
							}
							else if(Double.parseDouble(para2)==1 ) {
								System.out.println("RobotMove1");
								nToolMode=1;
							}
							else if(Double.parseDouble(para2)==3 ) {
								System.out.println("RobotMove3");
								nToolMode=3;
							}
							else if(Double.parseDouble(para2)==4 ) {
								System.out.println("RobotMove4");
								nToolMode=4;
							}
							else if(Double.parseDouble(para2)==5 ) {
								System.out.println("RobotMove5");
								nToolMode=5;
							}
							else if(Double.parseDouble(para2)==6 ) {
								System.out.println("RobotMove6");
								nToolMode=6;
							}
							else if(Double.parseDouble(para2)==7 ) {
								System.out.println("RobotMove7");
								nToolMode=7;
							}
							else if(Double.parseDouble(para2)==8 ) {
								System.out.println("RobotMove8");
								nToolMode=8;
							}
							else if(Double.parseDouble(para2)==9 ) {
								System.out.println("RobotMove9");
								nToolMode=9;
							}
							else if(Double.parseDouble(para2)==10 ) {
								System.out.println("RobotMove10");
								nToolMode=10;
							}
							else if(Double.parseDouble(para2)==11 ) {
								System.out.println("RobotMove11");
								nToolMode=11;
							}
							else if(Double.parseDouble(para2)==12 ) {
								System.out.println("RobotMove12");
								nToolMode=12;
							}
							else if(Double.parseDouble(para2)==13 ) {
								System.out.println("RobotMove13");
								nToolMode=13;
							}					
							else if(Double.parseDouble(para2)==14 ) {
								System.out.println("RobotMove14");
								nToolMode=14;
							}
							else if(Double.parseDouble(para2)==15 ) {
								System.out.println("RobotMove15");
								nToolMode=15;
							}
							else if(Double.parseDouble(para2)==16 ) {
								System.out.println("RobotMove16");
								nToolMode=16;
							}
							else if(Double.parseDouble(para2)==17 ) {
								System.out.println("RobotMove17");
								nToolMode=17;
							}
							else if(Double.parseDouble(para2)==18 ) {
								System.out.println("RobotMove18");
								nToolMode=18;
							}
							else if(Double.parseDouble(para2)==19 ) {
								System.out.println("RobotMove19");
								nToolMode=19;
							}
							else if(Double.parseDouble(para2)==20 ) {
								System.out.println("RobotMove20");
								nToolMode=20;
							}
							else if(Double.parseDouble(para2)==21 ) {
								System.out.println("RobotMove21");
								nToolMode=21;
							}
							else if(Double.parseDouble(para2)==22 ) {
								System.out.println("RobotMove22");
								nToolMode=22;
							}
							else if(Double.parseDouble(para2)==23 ) {
								System.out.println("RobotMove23");
								nToolMode=23;
							}
							else if(Double.parseDouble(para2)==24 ) {
								System.out.println("RobotMove24");
								nToolMode=24;
							}
							else if(Double.parseDouble(para2)==25 ) {
								System.out.println("RobotMove25");
								nToolMode=25;
							}
							else if(Double.parseDouble(para2)==26 ) {
								System.out.println("RobotMove26");
								nToolMode=26;
							}					
							else if(Double.parseDouble(para2)==27 ) {
								System.out.println("RobotMove27");
								nToolMode=27;
							}
							else if(Double.parseDouble(para2)==28 ) {
								System.out.println("RobotMove28");
								nToolMode=28;
							}					
							else if(Double.parseDouble(para2)==29 ) {
								System.out.println("RobotMove29");
								nToolMode=29;
							}
							else if(Double.parseDouble(para2)==30 ) {
								System.out.println("RobotMove30");
								nToolMode=30;
							}
							else if(Double.parseDouble(para2)==31 ) {
								System.out.println("RobotMove31");
								nToolMode=31;
							}
							else if(Double.parseDouble(para2)==32 ) {
								System.out.println("RobotMove32");
								nToolMode=32;
							}
							else if(Double.parseDouble(para2)==33 ) {
								System.out.println("RobotMove33");
								nToolMode=33;
							}
							else if(Double.parseDouble(para2)==34 ) {
								System.out.println("RobotMove34");
								nToolMode=34;
							}
							else if(Double.parseDouble(para2)==35 ) {
								System.out.println("RobotMove35");
								nToolMode=35;
							}
							else if(Double.parseDouble(para2)==36 ) {
								System.out.println("RobotMove36");
								nToolMode=36;
							}
							else if(Double.parseDouble(para2)==37 ) {
								System.out.println("RobotMove37");
								nToolMode=37;
							}
							else if(Double.parseDouble(para2)==38 ) {
								System.out.println("RobotMove38");
								nToolMode=38;
							}
							else if(Double.parseDouble(para2)==39 ) {
								System.out.println("RobotMove39");
								nToolMode=39;
							}
							else if(Double.parseDouble(para2)==40 ) {
								System.out.println("RobotMove40");
								nToolMode=40;
							}
							writer_recive.write("$res,RobotMove,0$");
							writer_recive.flush();
							
						}
						else if(units[1].equals("setwm")){
							String para3 = units[2].substring(0, units[2].length() - 1);
//							System.out.println("setwm: " + para3);
							nWorkingmode=Integer.parseInt(para3);
							writer_recive.write("$res,setwm"+nWorkingmode);
//							System.out.println("setwm:"+nWorkingmode);
							writer_recive.flush();
							
						}
						else if(units[1].equals("stcp")){
							System.out.println("normal " + units);
							String para4 = units[2].substring(0, units[2].length() - 1);
							System.out.println("stcp: " + para4);
//							nToolMode=Integer.parseInt(para4);
							writer_recive.write("$res,stcp,1");
							writer_recive.flush();
						}
						else{
							writer_recive.write("$res,err,-1");
							writer_recive.flush();
						}
					
					}
					ThreadUtil.milliSleep(20);
					
				}catch (IOException e) {
					System.out.println("closed.");					
					System.out.println("Socket.");
					
					in.close();
					writer_recive.close();
					outputStream_recive.close();
					socket_recive.close();
					serverSocket.close();
					
					writer_recive=null;
					outputStream_recive=null;
					socket_recive=null;
					serverSocket=null;
					in=null;
					bPause=true;
					
					
				}

			}
			System.out.println("closed11.");					
			System.out.println("Socket11.");
			
			if(in!=null){
				in.close();
				in=null;
				System.out.println("in");
			}
			if(writer_recive!=null){
				writer_recive.close();
				writer_recive=null;
				System.out.println("writer_recive");
			}
			if(outputStream_recive!=null ){
				outputStream_recive.close();
				outputStream_recive=null;
				System.out.println("outputStream_recive");
			}
			if(socket_recive!=null ){
				socket_recive.close();
				socket_recive=null;
				System.out.println("socket_recive");
			}
			if(serverSocket!=null ){
				serverSocket.close();
				serverSocket=null;
				System.out.println("serverSocket");
			}
			
			
			bPause=true;
			
			System.out.println("closed000");					
			System.out.println("Socket000");
			}catch(IOException e){
//				System.out.println("closed//...");		
				
				System.out.println("Socket//IOException e");
				ThreadUtil.milliSleep(1000);
			};
			}
		}
	}


	//////11111111

	public  class BreakTestProgram implements Callable<String> {
        
        
		
		public String call() {
			while (true){
				if(breaktest==true && SafeDataIO.getInput4()==true){
					breaktest=false;
				}
				
				if(breaktest==true && SafeDataIO.getInput4()==false){
					System.out.println("breaktest==true");
					BreakTest.initialize();
					BreakTest.run();
					System.out.println("BreakTest.run();");
					//Postion reference
					lbr.move(new PTP(new JointPosition(0.5,0.8,0.2,1.0,-0.5,-0.5,-1.5)).setJointVelocityRel(0.2));
					lbr.move(new PTP(new JointPosition(-0.5,-0.8,-0.2,-1.0,0.5,0.5,1.5)).setJointVelocityRel(0.2));
					lbr.move(new PTP(HOME).setJointVelocityRel(0.2));
					Err="100";
					breaktest=false;
				}
//				ThreadUtil.milliSleep(500);
//				System.out.println("new thread");
			}
			//return Err;	
			
		}
	}

	
	public HandGuidingMotion createhandGuidingMotion(){
		double min=0;
		double max=0;
		JointPosition jReady =lbr.getCurrentJointPosition();
		if(Math.toDegrees(jReady.get(JointEnum.J1)) < -10){
			min=Math.toDegrees(jReady.get(JointEnum.J1));
		}
		else
		{
			min=-10;
		}
		
		if(Math.toDegrees(jReady.get(JointEnum.J1)) > 10){
			max=Math.toDegrees(jReady.get(JointEnum.J1));
		}
		else
		{
			max=10;
		}
		
		HandGuidingMotion motion = new HandGuidingMotion();
		motion.setJointVelocityLimit(0.8)
		.setCartVelocityLimit(900.0).setJointLimitViolationFreezesAll(false)
		.setJointLimitsMax(Math.toRadians(max), Math.toRadians(30), Math.toRadians(45), Math.toRadians(100), Math.toRadians(65),Math.toRadians(-40), Math.toRadians(165))
		.setJointLimitsMin(Math.toRadians(min), Math.toRadians(-14), Math.toRadians(-45), Math.toRadians(0), Math.toRadians(-65),Math.toRadians(-100), Math.toRadians(-165))
		.setJointLimitsEnabled(true,true,true,true,true,true,true)


		

;
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




public  class motion implements Callable<String> {
	
	
	public void resetmotion()
	{
		System.out.println("start");
		io.setOutput6(true);
		ThreadUtil.milliSleep(200);
		System.out.println("dleay");
		io.setOutput6(false);
	}
	
    public void moveToInitialPosition()
    {
    	System.out.println("11");
        if (!ServoMotion.validateForImpedanceMode(lbr))
        {
            getLogger()
                    .info("Validation of torque model failed - correct your mass property settings");
            getLogger()
                    .info("Servo motion will be available for position controlled mode only, until validation is performed");
        }
    }
    
	//è½´é˜»æŠ—æŽ§åˆ¶ä»£ç �
    public void runSmartServoMotion(final IMotionControlMode controlMode)
    {

    	System.out.println("22");
        final boolean doDebugPrints = false;

        final JointPosition initialPosition = new JointPosition(
                lbr.getCurrentJointPosition());
        System.out.println("33");
        final SmartServo aSmartServoMotion = new SmartServo(initialPosition);

        // Set the motion properties to 10% of the systems abilities
        aSmartServoMotion.setJointAccelerationRel(0.1);
        aSmartServoMotion.setJointVelocityRel(0.1);
        System.out.println("44");
        aSmartServoMotion.setMinimumTrajectoryExecutionTime(20e-3);

        getLogger().info("Starting the SmartServo in " + controlMode);
        IMotionContainer container = _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(
                aSmartServoMotion.setMode(controlMode));
        // Fetch the Runtime of the Motion part
        final ISmartServoRuntime theSmartServoRuntime = aSmartServoMotion
                .getRuntime();

        // create an JointPosition Instance, to play with
        final JointPosition destination = new JointPosition(
                lbr.getJointCount());
        System.out.println("55");
        // For Roundtrip time measurement...
        final StatisticTimer timing = new StatisticTimer();
        try
        {
            // do a cyclic loop
            // Refer to some timing...
            // in nanosec
            final double omega = FREQENCY * 2 * Math.PI * 1e-9;
            final long startTimeStamp = System.nanoTime();
            final JointPosition currentPos = lbr.getCurrentJointPosition();

            while(SafeDataIO.getInput4()==false)
            {
            	final JointPosition currentPos_now = lbr.getCurrentJointPosition();
//            	System.out.println("xishu "+Math.toDegrees(Math.abs(Math.abs(Math.toRadians(170)) - Math.abs(currentPos_now.get(JointEnum.J5)))));
           
                // Timing - draw one step
                final OneTimeStep aStep = timing.newTimeStep();
                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // emulate some computational effort - or waiting for external
                // stuff
                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

                theSmartServoRuntime.updateWithRealtimeSystem();

                // Get the measured position in cartesian...
                final JointPosition curMsrJntPose = theSmartServoRuntime
                        .getAxisQMsrOnController();

                
   
                if(Math.abs(Math.toDegrees(currentPos_now.get(JointEnum.J5))) > 160 || Math.abs(Math.toDegrees(currentPos_now.get(JointEnum.J6))) > 110 || Math.abs(Math.toDegrees(currentPos_now.get(JointEnum.J7))) > 165){
                	 if (controlMode instanceof JointImpedanceControlMode)
                       {  		 
                		 
                       final JointImpedanceControlMode jointImp = (JointImpedanceControlMode) controlMode;
                       double para =200;
                       double para1=200;
                       double para2=200;
                       
                       if(Math.abs(Math.toDegrees(currentPos_now.get(JointEnum.J5))) > 160){
                    	   para = 2000 / Math.toDegrees(Math.abs(Math.abs(Math.toRadians(170)) - Math.abs(currentPos_now.get(JointEnum.J5))));  
//                    	   System.out.println(para);
                       }
                       else{
                    	   para=200;
                       }
                       
                       if(Math.abs(Math.toDegrees(currentPos_now.get(JointEnum.J6))) > 110){
                    	   para1 = 2000 / Math.toDegrees(Math.abs(Math.abs(Math.toRadians(120)) - Math.abs(currentPos_now.get(JointEnum.J6))));  
                       }
                       else{
                    	   para1=200;
                       }
                       
                       if(Math.abs(Math.toDegrees(currentPos_now.get(JointEnum.J7))) > 165){
                    	   para2 = 2000 / Math.toDegrees(Math.abs(Math.abs(Math.toRadians(175)) - Math.abs(currentPos_now.get(JointEnum.J7))));  
                       }
                       else{
                    	   para2=200;
                       }
                       
//                       System.out.println("xishu "+Math.toDegrees(Math.abs(Math.abs(Math.toRadians(170)) - Math.abs(currentPos_now.get(JointEnum.J5)))));
                       jointImp.setStiffness(2000, 2000, 2000, 2000, para, para1, para2);
//                       System.out.println(para);
                       jointImp.setDamping(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
                       theSmartServoRuntime
                               .changeControlModeSettings(jointImp);
                		 
                		 
                       }
                }
                else{
                
                    
                	if (controlMode instanceof JointImpedanceControlMode)
               	 	{
               	 		final JointImpedanceControlMode jointImp = (JointImpedanceControlMode) controlMode;
               	 		jointImp.setStiffness(2000, 2000, 2000, 2000, 200, 200, 200);
               	 		jointImp.setDamping(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
               	 		// Send the new Stiffness settings down to the
               	 		// controller
               	 		theSmartServoRuntime
               	 		.changeControlModeSettings(jointImp);
               	 	} 	
                }
                
                
                
                destination.set(currentPos);
                
                theSmartServoRuntime
                        .setDestination(destination);

                aStep.end();


            }
        }
        catch (final Exception e)
        {
            getLogger().info(e.getLocalizedMessage());
            e.printStackTrace();
        }

        //Print statistics and parameters of the motion
        getLogger().info("Displaying final states after loop "
                + controlMode.getClass().getName());
        getLogger().info(getClass().getName() + theSmartServoRuntime.toString());
        // Stop the motion
        theSmartServoRuntime.stopMotion();
        getLogger().info("Statistic Timing of Overall Loop " + timing);
        if (timing.getMeanTimeMillis() > 150)
        {
            getLogger().info("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
            getLogger().info("Under Windows, you should play with the registry, see the e.g. the RealtimePTP Class javaDoc for details");
        }

    }
	 
	    protected JointImpedanceControlMode createJointImp()
	    {
	        final JointImpedanceControlMode jointImp = new JointImpedanceControlMode(2000, 2000, 2000, 2000, 200, 200, 200);
//	        jointImp.setStiffness(1, 1, 1, 1, 1, 1, 1);
	   
	        jointImp.setDamping(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
	        return jointImp;
	    }
	    
	    protected CartesianImpedanceControlMode ConeLimit()
	    {
	        final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
	        cartImp.parametrize(CartDOF.X).setStiffness(5000.0);
	        cartImp.parametrize(CartDOF.Y).setStiffness(5000.0);
	        cartImp.parametrize(CartDOF.Z).setStiffness(500.0);
	        cartImp.parametrize(CartDOF.ROT).setStiffness(210.0);

//	        cartImp.parametrize(CartDOF.X).setAdditionalControlForce(-4.9);
	        cartImp.setNullSpaceStiffness(100.);
	   
//	        cartImp.setMaxCartesianVelocity(5.0,5.0,5.0,0.2,0.2, 0.2);
	        // For your own safety, shrink the motion abilities to useful limits
//	        cartImp.setMaxControlForce(100.0, 100.0, 50.0, 20.0, 20.0, 20.0, true);
//	        cartImp.setMaxControlForce(100.0, 100.0, 50.0, 20.0, 20.0, 20.0, true);
//	        cartImp.setMaxControlForce(1,z 1, 1, 1, 1, 1, true);
	        cartImp.setMaxPathDeviation(150., 150., 150., 3., 3., 3.);
	        return cartImp; 	
	    }
	    protected CartesianImpedanceControlMode InstallLimit()
	    {
	        final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
	        cartImp.parametrize(CartDOF.X).setStiffness(5000.0);
	        cartImp.parametrize(CartDOF.Y).setStiffness(5000.0);
	        cartImp.parametrize(CartDOF.Z).setStiffness(500.0);
	        cartImp.parametrize(CartDOF.ROT).setStiffness(300.0);
	        
	        cartImp.setNullSpaceStiffness(100.);
	        cartImp.setMaxPathDeviation(150., 150., 150., 3., 3., 3.);
	        return cartImp; 	
	    }
	    protected CartesianImpedanceControlMode ZoneLimit()
	    {
	        final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
	        cartImp.parametrize(CartDOF.X).setStiffness(4000.0);
	        cartImp.parametrize(CartDOF.Y).setStiffness(4000.0);
	        cartImp.parametrize(CartDOF.Z).setStiffness(2000.0);
	        cartImp.parametrize(CartDOF.ROT).setStiffness(300.0);

//	        cartImp.parametrize(CartDOF.X).setAdditionalControlForce(-4.9);
	        cartImp.setNullSpaceStiffness(100.);
	   
//	        cartImp.setMaxCartesianVelocity(5.0,5.0,5.0,0.2,0.2, 0.2);
	        // For your own safety, shrink the motion abilities to useful limits
//	        cartImp.setMaxControlForce(100.0, 100.0, 50.0, 20.0, 20.0, 20.0, true);
//	        cartImp.setMaxControlForce(100.0, 100.0, 50.0, 20.0, 20.0, 20.0, true);
//	        cartImp.setMaxControlForce(1,z 1, 1, 1, 1, 1, true);
	        cartImp.setMaxPathDeviation(150., 150., 150., 3., 3., 3.);
	        return cartImp; 	
	    }
	    protected CartesianImpedanceControlMode createCartImp()
	    {
	        final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
	        cartImp.parametrize(CartDOF.TRANSL).setStiffness(1000.0);
	        cartImp.parametrize(CartDOF.ROT).setStiffness(100.0);
//	        cartImp.parametrize(CartDOF.X).setAdditionalControlForce(-4.9);
	        cartImp.setNullSpaceStiffness(100.);
	        
//	        cartImp.setMaxCartesianVelocity(5.0,5.0,5.0,0.2,0.2, 0.2);
	        // For your own safety, shrink the motion abilities to useful limits
	        cartImp.setMaxPathDeviation(150., 150., 50., 50., 50., 50.);
	        return cartImp;
	    }
	    
	 //ç¬›å�¡å°”é˜»æŠ—æŽ§åˆ¶ä»£ç �
	    public void runSmartCartesianMotion(final IMotionControlMode controlMode){
//	     	_loadData = new LoadData();
//	        _loadData.setMass(MASS);
//	        _loadData.setCenterOfMass(
//	                CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
//	                CENTER_OF_MASS_IN_MILLIMETER[2]);
//	        _toolAttachedToLBR = new Tool("Tool", _loadData);
//
//	        XyzAbcTransformation trans = XyzAbcTransformation.ofTranslation(
//	                TRANSLATION_OF_TOOL[0], TRANSLATION_OF_TOOL[1],
//	                TRANSLATION_OF_TOOL[2]);
//	        ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME
//	                + "(TCP)", trans);
//	        _toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
//	        // Attach tool to the robot
//	        _toolAttachedToLBR.attachTo(lbr.getFlange());
	        
	        final JointPosition initialPosition = new JointPosition(
	                lbr.getCurrentJointPosition());

	        final DirectServo aDirectServoMotion = new DirectServo(initialPosition);

	        aDirectServoMotion.setMinimumTrajectoryExecutionTime(40e-3);

	        getLogger().info("Starting the DirectServo motion in " + controlMode);
	        _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(
	                aDirectServoMotion.setMode(controlMode));
	        System.out.println("sss ");
	        final IDirectServoRuntime theServoRuntime = aDirectServoMotion
	                .getRuntime();
	        System.out.println("ggg ");
	        // create an JointPosition Instance, to play with
	        final JointPosition destination = new JointPosition(
	                lbr.getJointCount());
	        System.out.println("vvv ");
	        // For Roundtrip time measurement...
	        final StatisticTimer timing = new StatisticTimer();
	        System.out.println("ccc");
	        try
	        {
	            // do a cyclic loop
	            // Refer to some timing...
	            // in nanosec
	            // freqency * ...
	            final double omega = FREQENCY * 2 * Math.PI * 1e-9;
	            final long startTimeStamp = System.nanoTime();

	            //for (int i = 0; i < NUM_RUNS; ++i)
	         	final JointPosition currentPos = lbr.getCurrentJointPosition();
	         	
	         	JointPosition currentPos_CheckSafety;
//	         	destination.set(currentPos_test);
	         	destination.set(currentPos);
	         	
	         	boolean bDanger=false;
//	         	System.out.println("hhh");
	         	System.out.println(bDangerous+"dd"+nWorkingmode);
	            while (nWorkingmode==4 && bDangerous==false)
	            {
	          
	            	currentPos_CheckSafety=lbr.getCurrentJointPosition();
	            	if (Math.abs(Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J1))) < 160 && Math.abs(Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J2))) < 110 && Math.abs(Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J3))) < 160 && Math.abs(Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J4))) < 110 && Math.abs(Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J5))) < 160 && Math.abs(Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J6))) < 110 && Math.abs(Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J7))) < 165){
	            		bDangerous=false;
	            		
	            	}
	            	else{
	            		System.out.println("J1"+Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J1)));
	            		System.out.println("J2"+Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J2)));
	            		System.out.println("J3"+Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J3)));
	            		System.out.println("J4"+Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J4)));
	            		System.out.println("J5"+Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J5)));
	            		System.out.println("J6"+Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J6)));
	            		System.out.println("J7"+Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J7)));
	            		getLogger().info("bDangerous==true");
	            		
	            		bDangerous=true;
	            	}
	            	
	                theServoRuntime.setDestination(destination);
	                     
//	                // Timing - draw one step
//	                final OneTimeStep aStep = timing.newTimeStep();
//	                // ///////////////////////////////////////////////////////
//	                // Insert your code here
//	                // e.g Visual Servoing or the like
//	                // emulate some computational effort - or waiting for external
//	                // stuff
//	                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
	//
//	                theServoRuntime.updateWithRealtimeSystem();
	//
//	                final double curTime = System.nanoTime() - startTimeStamp;
//	                final double sinArgument = omega * curTime;
	//
//	                for (int k = 0; k < destination.getAxisCount(); ++k)
//	                {
//	                    destination.set(k, Math.sin(sinArgument)
//	                            * AMPLITUDE + initialPosition.get(k));
//	                }
//	                theServoRuntime
//	                        .setDestination(destination);
	//
//	                // Modify the stiffness settings every now and then               
//	                if (i % (NUM_RUNS / 10) == 0)
//	                {
//	                    // update realtime system
//	                    if (controlMode instanceof CartesianImpedanceControlMode)
//	                    {
//	                        final CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) controlMode;
//	                        final double aTransStiffVal = Math.max(100. * (i
//	                                / (double) NUM_RUNS + 1), 1000.);
//	                        final double aRotStiffVal = Math.max(10. * (i
//	                                / (double) NUM_RUNS + 1), 150.);
//	                        cartImp.parametrize(CartDOF.TRANSL).setStiffness(aTransStiffVal);
//	                        cartImp.parametrize(CartDOF.ROT).setStiffness(aRotStiffVal);
//	                        // Send the new Stiffness settings down to the
//	                        // controller
//	                        theServoRuntime
//	                                .changeControlModeSettings(cartImp);
//	                    }
//	                }
//	                aStep.end();
	            }
	        }
	        catch (final Exception e)
	        {
	            getLogger().info(e.getLocalizedMessage());
	            e.printStackTrace();
	        }

	        // Print statistics and parameters of the motion
	        getLogger().info("Displaying final states after loop "
	                + controlMode.getClass().getName());

	        getLogger().info(getClass().getName() + "\n" + theServoRuntime.toString());

	        // Stop the motion
	        theServoRuntime.stopMotion();
	        getLogger().info("Statistic Timing of Overall Loop " + timing);
	        if (timing.getMeanTimeMillis() > 150)
	        {
	            getLogger().info("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
	            getLogger().info("You should check the TCP/IP Stack Configuration - see the manual for details");
	        }
	    }

	    private StatisticTimer startSineMovement(
	            ISmartServoLINRuntime smartServoLINRuntime, StatisticTimer timing)
	    {
	        Frame aFrame = smartServoLINRuntime.getCurrentCartesianDestination(lbr
	                .getFlange());

	        getLogger().info("Do sine movement");
	        try
	        {
	            double omega = FREQENCY * 2 * Math.PI * 1e-9;
	            long startTimeStamp = System.nanoTime();
	        	Frame Ptest2 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"));
	         	
	           while(nWorkingmode==2)
	            {
	            
	                   //testdata x:735  y:7.59  z:122 Aï¼š-91 Bï¼š-40 Cï¼š-178 $cmd,ml,715,7,122,-91,-40,-178$
						//$cmd,RobotMove,1$
						if(nX!=0 && nY!=0 && nZ!=0 ){
							Ptest2.setX(nX);
							Ptest2.setY(nY);
							Ptest2.setZ(nZ);
//							System.out.println("nx:"+nX+"  ny:"+nY+"  nz:"+nZ+"  a:"+nZ);
							Ptest2.setAlphaRad(Math.toRadians(nA));
							Ptest2.setBetaRad(Math.toRadians(nB));
							Ptest2.setGammaRad(Math.toRadians(nC));
							
//							ThreadUtil.milliSleep(500);
//							System.out.println("222");
						}
						
	                final OneTimeStep aStep = timing.newTimeStep();
	                // ///////////////////////////////////////////////////////
	                // Insert your code here
	                // e.g Visual Servoing or the like
	                // Synchronize with the realtime system

	                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

	                // Update the smart servo LIN runtime
	                smartServoLINRuntime.updateWithRealtimeSystem();

	                double curTime = System.nanoTime() - startTimeStamp;
	                double sinArgument = omega * curTime;

	                // Compute the sine function
	                Frame destFrame = new Frame(aFrame);
	                destFrame.setZ(AMPLITUDE * Math.sin(sinArgument));

	                // Set new destination
	                smartServoLINRuntime.setDestination(Ptest2);
	                aStep.end();
	            }

	        }
	        catch (Exception e)
	        {
	            getLogger().error(e.getLocalizedMessage());
	            e.printStackTrace();
	        }
	        return timing;
	    }
	    
	
//	@SuppressWarnings("null")
	public String call() {
//		int answer;
//		answer = getApplicationUI().displayModalDialog(
//		ApplicationDialogType.INFORMATION,"Moving Mode", "Manule","Handle");
		boolean DangerMove=false;
		int nLastWorkingmode=0;
		while (true)
		{ 
//			boolean btest=SafeDataIO.getInput4();
//			if (btest==true)
//			{
//				nWorkingmode=1;
//				System.out.println("SafeDataIO.getInput4"+btest);
//			}
//			System.out.println(btest);
			boolean btest=SafeDataIO.getInput4();
			if (btest==true)
			{
				nWorkingmode=1;
			}
			if (nWorkingmode==1){
				Err="0,";
				System.out.println("start handguiding");
				ThreadUtil.milliSleep(500);
		    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/test"));
		    	needle.getFrame("/test").move(ptp(Ptest1).setJointVelocityRel(0.2));
				nLastWorkingmode=nWorkingmode;
				if(SafeDataIO.getInput4()==true)
				{
					DangerMove=false;
						//其他指的是打磨模式
						needle.getFrame("/zuo_21002_zhiPolish").move(createhandGuidingMotion());
						bDangerous=false;
						nWorkingmode=0;
//					}
					
	                nX=0;
	                nY=0;
	                nZ=0;
	                nA=0;
	                nB=0;
	                nC=0;
				}
				nWorkingmode=0;
				
			}
		    else if (nWorkingmode==2 ){
		    	nLastWorkingmode=nWorkingmode;
		    	DangerMove=false;
//				System.out.println("automode"+nWorkingmode);
//				Frame Ptest1= getApplicationData().getFrame("/P1").copyWithRedundancy();	
		    	ThreadUtil.milliSleep(1000);
		    	final CartesianImpedanceControlMode cartImp = createCartImp();	
		    	if(nX==1)
		    	{
		    		System.out.println("zhunbei_ready");
		    		needle.getFrame("/zuo_21002_zhiPolish").move(new PTP(jointPos).setJointVelocityRel(0.2));		    		
					Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"));
		    	
		    	}
		    	//左侧
		    	else if(nX==2)
		    	{
		    		System.out.println("zuoce_ready");
		    		lbr.moveAsync(new PTP(jointPos).setJointVelocityRel(0.2).setMode(cartImp));
		    		lbr.moveAsync(new PTP(jointPos_zuo).setJointVelocityRel(0.2).setMode(cartImp));
		    	}
		    	//右侧
		    	else if(nX==3)
		    	{
		    		System.out.println("youce_ready");
		    		lbr.moveAsync(new PTP(jointPos).setJointVelocityRel(0.2).setMode(cartImp));
		    		lbr.moveAsync(new PTP(jointPos_you).setJointVelocityRel(0.2).setMode(cartImp));
		    	}
		    	else{
		    		System.out.println("err");
		    	}
	                System.out.println("*2");
	                nWorkingmode=0;
	                System.out.println("*3");
	                nX=0;
	                nY=0;
	                nZ=0;
	                nA=0;
	                nB=0;
	                nC=0;
			}
			
			else if(nWorkingmode==3){
				DangerMove=false;
				nLastWorkingmode=nWorkingmode;
				
				
				Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"));

               //testdata x:735  y:7.59  z:122 Aï¼š-91 Bï¼š-40 Cï¼š-178 $cmd,ml,715,7,122,-91,-40,-178$
				//$cmd,RobotMove,1$
				if(nX!=0 && nY!=0 && nZ!=0){
					Ptest1.setX(nX);
					Ptest1.setY(nY);
					Ptest1.setZ(nZ);
//					System.out.println("nx:"+nX+"  ny:"+nY+"  nz:"+nZ+"  a:"+nZ);
					Ptest1.setAlphaRad(Math.toRadians(nA));
					Ptest1.setBetaRad(Math.toRadians(nB));
					Ptest1.setGammaRad(Math.toRadians(nC));

					
					if(Math.abs(nX)<2000 && Math.abs(nY)<2000 && Math.abs(nZ)<2000 && Math.abs(nA)<2000 && Math.abs(nB)<2000 && Math.abs(nC)<2000){
						//System.out.println("pre_Place11***:"+pre_Place);
						needle.getFrame("/zuo_21002_zhiPolish").move(ptp(pre_Place).setJointVelocityRel(0.35));	
					}
					else{
						//System.out.println("Err_DangerPlace: "+"nX:"+nX+"nY:"+nY+"nZ:"+nZ+"nA:"+nA+"nB:"+nB+"nC:"+nC);
					}
					
					
//					ThreadUtil.milliSleep(500);
//					System.out.println("222");
				}
				else{
					ThreadUtil.milliSleep(10);
				}
				//nWorkingmode=0;

			}
			else if(nWorkingmode==4){
				nLastWorkingmode=nWorkingmode;
		        // Initialize Cartesian impedance mode    
//				System.out.println("CartesianimplentMode"+nWorkingmode);
//				moveToInitialPosition();
//				final CartesianImpedanceControlMode cartImp = createCartImp();
//		        runSmartCartesianMotion(cartImp);
           
		    	//圆锥打磨模式
		    	final CartesianImpedanceControlMode cartImp = ConeLimit();	
		    		
		    	System.out.println("ConeLimit");
		    	JointPosition jReady =lbr.getCurrentJointPosition();
//                final JointPosition currentPos = lbr.getCurrentJointPosition();
                
     		   
                if(Math.toDegrees(jReady.get(JointEnum.J1)) < -160 || Math.toDegrees(jReady.get(JointEnum.J2)) < -34 || Math.toDegrees(jReady.get(JointEnum.J3)) < -65 || Math.toDegrees(jReady.get(JointEnum.J4)) < -10 || Math.toDegrees(jReady.get(JointEnum.J5)) < -160 ||  Math.toDegrees(jReady.get(JointEnum.J6)) < -110 || Math.toDegrees(jReady.get(JointEnum.J7)) < -165 || Math.toDegrees(jReady.get(JointEnum.J1)) > 160 || Math.toDegrees(jReady.get(JointEnum.J2)) > 70 || Math.toDegrees(jReady.get(JointEnum.J3)) > 55 || Math.toDegrees(jReady.get(JointEnum.J4)) > 110 || Math.toDegrees(jReady.get(JointEnum.J5)) > 160 ||  Math.toDegrees(jReady.get(JointEnum.J6)) > 110 || Math.toDegrees(jReady.get(JointEnum.J7)) > 165){
                	if(Math.toDegrees(jReady.get(JointEnum.J1)) < -160 || Math.toDegrees(jReady.get(JointEnum.J1)) > 160){
                		System.out.println("J1:"+jReady.get(JointEnum.J1)+"   MAX:160;MIN:-160");
                	}
                	if(Math.toDegrees(jReady.get(JointEnum.J2)) < -34 || Math.toDegrees(jReady.get(JointEnum.J2)) > 70){
                		System.out.println("J2:"+jReady.get(JointEnum.J2)+"   MAX:70;MIN:-34");
                	}
                	if(Math.toDegrees(jReady.get(JointEnum.J3)) < -65 || Math.toDegrees(jReady.get(JointEnum.J3)) > 55){
                		System.out.println("J3:"+jReady.get(JointEnum.J3)+"   MAX:55;MIN:-65");
                	}
                	if(Math.toDegrees(jReady.get(JointEnum.J4)) < -10 || Math.toDegrees(jReady.get(JointEnum.J4)) > 115){
                		System.out.println("J4:"+jReady.get(JointEnum.J4)+"   MAX:110;MIN:-10");
                	}
                	if(Math.toDegrees(jReady.get(JointEnum.J5)) < -160 || Math.toDegrees(jReady.get(JointEnum.J5)) > 160){
                		System.out.println("J5:"+jReady.get(JointEnum.J5)+"   MAX:160;MIN:-160");
                	}
                	if(Math.toDegrees(jReady.get(JointEnum.J6)) < -110 || Math.toDegrees(jReady.get(JointEnum.J6)) > 110){
                		System.out.println("J6:"+jReady.get(JointEnum.J6)+"   MAX:110;MIN:-110");
                	}
                	if(Math.toDegrees(jReady.get(JointEnum.J7)) < -165 || Math.toDegrees(jReady.get(JointEnum.J7)) > 165){
                		System.out.println("J7:"+jReady.get(JointEnum.J7)+"   MAX:165;MIN:-165");
                	}
                	System.out.println("dangermove1");
                	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"));
			    	needle.getFrame("/zuo_21002_zhiPolish").move(ptp(Ptest1).setJointVelocityRel(0.2));
			    	DangerMove=true;
                }
                else{
                	
                	if (nToolMode==1){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiPolish"));
    			    	needle.getFrame("/zuo_21001_zhiPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==2){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanPolish"));
    			    	needle.getFrame("/zuo_21001_wanPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==3){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiSet"));
    			    	needle.getFrame("/zuo_21001_zhiSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==4){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanSet"));
    			    	needle.getFrame("/zuo_21001_zhiSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==5){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiPolish"));
    			    	needle.getFrame("/you_21001_zhiPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==6){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanPolish"));
    			    	needle.getFrame("/you_21001_wanPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==7){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiSet"));
    			    	needle.getFrame("/you_21001_zhiSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==8){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanSet"));
    			    	needle.getFrame("/you_21001_wanSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if (nToolMode==9){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"));
    			    	needle.getFrame("/zuo_21002_zhiPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==10){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanPolish"));
    			    	needle.getFrame("/zuo_21002_wanPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==11){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiSet"));
    			    	needle.getFrame("/zuo_21002_zhiSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==12){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanSet"));
    			    	needle.getFrame("/zuo_21002_zhiSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==13){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiPolish"));
    			    	needle.getFrame("/you_21002_zhiPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==14){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanPolish"));
    			    	needle.getFrame("/you_21002_wanPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==15){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiSet"));
    			    	needle.getFrame("/you_21002_zhiSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==16){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanSet"));
    			    	needle.getFrame("/you_21002_wanSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if (nToolMode==17){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiPolish"));
    			    	needle.getFrame("/zuo_21003_zhiPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==18){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanPolish"));
    			    	needle.getFrame("/zuo_21003_wanPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==19){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiSet"));
    			    	needle.getFrame("/zuo_21003_zhiSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==20){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanSet"));
    			    	needle.getFrame("/zuo_21003_zhiSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==21){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiPolish"));
    			    	needle.getFrame("/you_21003_zhiPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==22){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanPolish"));
    			    	needle.getFrame("/you_21003_wanPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==23){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiSet"));
    			    	needle.getFrame("/you_21003_zhiSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==24){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanSet"));
    			    	needle.getFrame("/you_21003_wanSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if (nToolMode==25){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiPolish"));
    			    	needle.getFrame("/zuo_21004_zhiPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==26){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanPolish"));
    			    	needle.getFrame("/zuo_21004_wanPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==27){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiSet"));
    			    	needle.getFrame("/zuo_21004_zhiSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==28){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanSet"));
    			    	needle.getFrame("/zuo_21004_zhiSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==29){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiPolish"));
    			    	needle.getFrame("/you_21004_zhiPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==30){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanPolish"));
    			    	needle.getFrame("/you_21004_wanPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==31){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiSet"));
    			    	needle.getFrame("/you_21004_zhiSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==32){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanSet"));
    			    	needle.getFrame("/you_21004_wanSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if (nToolMode==33){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiPolish"));
    			    	needle.getFrame("/zuo_21005_zhiPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==34){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanPolish"));
    			    	needle.getFrame("/zuo_21005_wanPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==35){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiSet"));
    			    	needle.getFrame("/zuo_21005_zhiSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==36){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanSet"));
    			    	needle.getFrame("/zuo_21005_zhiSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==37){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiPolish"));
    			    	needle.getFrame("/you_21005_zhiPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==38){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanPolish"));
    			    	needle.getFrame("/you_21005_wanPolish").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==39){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiSet"));
    			    	needle.getFrame("/you_21005_zhiSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
    				else if(nToolMode==40){
    			    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanSet"));
    			    	needle.getFrame("/you_21005_wanSet").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
    				}
                }
                

                
		        // Initialize Joint impedance mode    
//				System.out.println("JointimplentMode"+nWorkingmode);
//				moveToInitialPosition();
//		        final JointImpedanceControlMode jointImp = createJointImp();
//		        runSmartServoMotion(jointImp);
                
                

		        nWorkingmode=0;

				
		        
		  if(bDangerous==true){
			  nWorkingmode=1;
			  System.out.println("WorkingModeForceToSet1");
		  }

		}
			//自动矫正点位
			//自动矫正点位
			else if(nWorkingmode==5){
				Frame Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiSet"));;
//				System.out.println("StartAuto");
//				ThreadUtil.milliSleep(1000);
//				Frame current = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiSet"));
            	if (nToolMode==1){
            		Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiPolish"));
				}
				else if(nToolMode==2){
			    	Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanPolish"));
				}
				else if(nToolMode==3){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiSet"));
				}
				else if(nToolMode==4){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanSet"));
				}
				else if(nToolMode==5){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiPolish"));
				}
				else if(nToolMode==6){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanPolish"));
				}
				else if(nToolMode==7){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiSet"));
				}
				else if(nToolMode==8){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanSet"));
				}
				else if (nToolMode==9){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"));
				}
				else if(nToolMode==10){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanPolish"));
				}
				else if(nToolMode==11){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiSet"));
				}
				else if(nToolMode==12){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanSet"));
				}
				else if(nToolMode==13){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiPolish"));
				}
				else if(nToolMode==14){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanPolish"));
				}
				else if(nToolMode==15){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiSet"));
				}
				else if(nToolMode==16){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanSet"));
				}
				else if (nToolMode==17){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiPolish"));
				}
				else if(nToolMode==18){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanPolish"));
				}
				else if(nToolMode==19){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiSet"));
				}
				else if(nToolMode==20){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanSet"));
				}
				else if(nToolMode==21){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiPolish"));
				}
				else if(nToolMode==22){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanPolish"));
				}
				else if(nToolMode==23){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiSet"));
				}
				else if(nToolMode==24){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanSet"));
				}
				else if (nToolMode==25){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiPolish"));
				}
				else if(nToolMode==26){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanPolish"));
				}
				else if(nToolMode==27){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiSet"));
				}
				else if(nToolMode==28){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanSet"));
				}
				else if(nToolMode==29){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiPolish"));
				}
				else if(nToolMode==30){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanPolish"));
				}
				else if(nToolMode==31){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiSet"));
				}
				else if(nToolMode==32){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanSet"));
				}
				else if (nToolMode==33){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiPolish"));
				}
				else if(nToolMode==34){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanPolish"));
				}
				else if(nToolMode==35){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiSet"));
				}
				else if(nToolMode==36){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanSet"));
				}
				else if(nToolMode==37){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiPolish"));
				}
				else if(nToolMode==38){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanPolish"));
				}
				else if(nToolMode==39){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiSet"));
				}
				else if(nToolMode==40){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanSet"));
				}
				int num=0;
				double nMinA=0,nMinB=0,nMinC=0;
				double nMinSun=10000000;
				double nObjectA=10000,nObjectB=100000,nObjectC=100000;
				double nObjectA_BackUp=10000,nObjectB_BackUp=100000,nObjectC_BackUp=100000;
//				Frame destObject1 = current.setX(current.getX());
//				System.out.println("current.getX():"+current.getX());
//				System.out.println("current.getA():"+current.getAlphaRad());
//				System.out.println("current.getB():"+current.getBetaRad());
//				System.out.println("nA:"+nA+"  nB:"+nB+"  nC:"+nC);
				for (num = 0; num < 360; num = num + 1){
					//当前点位
					Frame current = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiSet"));
					
					if (nToolMode==1){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiPolish"), cmdPos2);
					}
					else if(nToolMode==2){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanPolish"), cmdPos2);
					}
					else if(nToolMode==3){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiSet"), cmdPos2);
					}
					else if(nToolMode==4){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanSet"), cmdPos2);
					}
					else if(nToolMode==5){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiPolish"), cmdPos2);
					}
					else if(nToolMode==6){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanPolish"), cmdPos2);
					}
					else if(nToolMode==7){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiSet"), cmdPos2);
					}
					else if(nToolMode==8){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanSet"), cmdPos2);
					}
					else if (nToolMode==9){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"), cmdPos2);
					}
					else if(nToolMode==10){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanPolish"), cmdPos2);
					}
					else if(nToolMode==11){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiSet"), cmdPos2);
					}
					else if(nToolMode==12){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanSet"), cmdPos2);
					}
					else if(nToolMode==13){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiPolish"), cmdPos2);
					}
					else if(nToolMode==14){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanPolish"), cmdPos2);
					}
					else if(nToolMode==15){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiSet"), cmdPos2);
					}
					else if(nToolMode==16){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanSet"), cmdPos2);
					}
					else if (nToolMode==17){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiPolish"), cmdPos2);
					}
					else if(nToolMode==18){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanPolish"), cmdPos2);
					}
					else if(nToolMode==19){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiSet"), cmdPos2);
					}
					else if(nToolMode==20){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanSet"), cmdPos2);
					}
					else if(nToolMode==21){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiPolish"), cmdPos2);
					}
					else if(nToolMode==22){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanPolish"), cmdPos2);
					}
					else if(nToolMode==23){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiSet"), cmdPos2);
					}
					else if(nToolMode==24){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanSet"), cmdPos2);
					}
					else if (nToolMode==25){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiPolish"), cmdPos2);
					}
					else if(nToolMode==26){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanPolish"), cmdPos2);
					}
					else if(nToolMode==27){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiSet"), cmdPos2);
					}
					else if(nToolMode==28){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanSet"), cmdPos2);
					}
					else if(nToolMode==29){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiPolish"), cmdPos2);
					}
					else if(nToolMode==30){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanPolish"), cmdPos2);
					}
					else if(nToolMode==31){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiSet"), cmdPos2);
					}
					else if(nToolMode==32){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanSet"), cmdPos2);
					}
					else if (nToolMode==33){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiPolish"), cmdPos2);
					}
					else if(nToolMode==34){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanPolish"), cmdPos2);
					}
					else if(nToolMode==35){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiSet"), cmdPos2);
					}
					else if(nToolMode==36){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanSet"), cmdPos2);
					}
					else if(nToolMode==37){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiPolish"), cmdPos2);
					}
					else if(nToolMode==38){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanPolish"), cmdPos2);
					}
					else if(nToolMode==39){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiSet"), cmdPos2);
					}
					else if(nToolMode==40){
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						current=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanSet"), cmdPos2);
					}
					Frame destObject = current.setX(current.getX());
					destObject = Object.setY(current.getY());
					destObject = Object.setZ(current.getZ());
					destObject = Object.setGammaRad(current.getGammaRad());		
					destObject = Object.setBetaRad(current.getBetaRad());
					destObject = Object.setAlphaRad(current.getAlphaRad());
					
					
//					Frame destObject = current.setX(-737.37);
//					destObject = Object.setY(216.62);
//					destObject = Object.setZ(232.57);
//					destObject = Object.setAlphaRad(-2.86530);
//					destObject = Object.setBetaRad(-0.096691);
//					destObject = Object.setGammaRad(2.3420570);		
					
				
					
					//System.out.println(Math.toDegrees(destObject.getBetaRad()));
					
					
					
//					nA=-157.22;
//					nB=22.9288;
//					nC=157.359;
//					nA=164.7;
//					nB=-31.71;
//					nC=-164.418;
//					System.out.println("nA:"+nA+" nB:"+nB+" nC:"+nC);
					
//					System.out.println("pre_Place"+pre_Place);
//					System.out.println("nX"+nX+"  nY"+nY+"  nZ"+nZ+"  nA"+nA+"  nB"+nB+"  nC"+nC);
					
					
					//目标点位
					Frame destFrame = current.setX(nX);
					destFrame = current.setY(nY);
					destFrame = current.setZ(nZ);
					destFrame=  current.setAlphaRad(Math.toRadians(nA));
					destFrame=  current.setBetaRad(Math.toRadians(nB));
					destFrame=  current.setGammaRad(Math.toRadians(nC));
					
//					System.out.println("11"+destFrame);
//					
//					destFrame = current.setX(-714.741);
//					destFrame = current.setY(144.732);
//					destFrame = current.setZ(315.285);
//					destFrame = current.setAlphaRad(Math.toRadians(-139.612));
//					destFrame = current.setBetaRad(Math.toRadians(39.7936));
//					destFrame = current.setGammaRad(Math.toRadians(154.836));	
//					System.out.println("22"+destFrame);
					
					
					Frame Ptest1 = destFrame.transform((Transformation.ofDeg(0, 0, 0, num, 0, 0)));
					
					if(Math.toDegrees(Ptest1.getAlphaRad())>174 &&Math.toDegrees(Ptest1.getAlphaRad())<179){
						nObjectA_BackUp=Math.toDegrees(Ptest1.getAlphaRad());
						nObjectB_BackUp=Math.toDegrees(Ptest1.getBetaRad());
						nObjectC_BackUp=Math.toDegrees(Ptest1.getGammaRad());
					}
//					nMinA=Math.abs(Math.abs(Math.toDegrees(destObject.getAlphaRad()))-Math.abs(Math.toDegrees(Ptest1.getAlphaRad())));
//					nMinB=Math.abs(Math.abs(Math.toDegrees(destObject.getBetaRad()))-Math.abs(Math.toDegrees(Ptest1.getBetaRad())));
//					nMinC=Math.abs(Math.abs(Math.toDegrees(destObject.getGammaRad()))-Math.abs(Math.toDegrees(Ptest1.getGammaRad())));
					
					nMinA=Math.abs(Math.toDegrees(destObject.getAlphaRad())-Math.toDegrees(Ptest1.getAlphaRad()));
					nMinB=Math.abs(Math.toDegrees(destObject.getBetaRad())-Math.toDegrees(Ptest1.getBetaRad()));
					nMinC=Math.abs(Math.toDegrees(destObject.getGammaRad())-Math.toDegrees(Ptest1.getGammaRad()));
					
					//System.out.println(Math.toDegrees(destObject.getAlphaRad()));
					//System.out.println(Math.toDegrees(Ptest1.getAlphaRad()));
					
					//System.out.println("a:"+nMinA+" b:"+nMinB+" c:"+nMinC);
					if((nMinA+nMinB+nMinC)<nMinSun)
					{
						
						nMinSun=(nMinA+nMinB+nMinC);
						nObjectA=Math.toDegrees(Ptest1.getAlphaRad());
						nObjectB=Math.toDegrees(Ptest1.getBetaRad());
						nObjectC=Math.toDegrees(Ptest1.getGammaRad());
					}
				
					//ThreadUtil.milliSleep(10);
				}
//				System.out.println((nMinSun));
				//System.out.println("a:"+Math.toDegrees(Ptest1.getAlphaRad())+" b:"+Math.toDegrees(Ptest1.getBetaRad())+" c:"+Math.toDegrees(Ptest1.getGammaRad()));
//				System.out.println("a:"+nObjectA+" b:"+nObjectB+" c:"+nObjectC);
//				System.out.println("a_BackUp:"+nObjectA_BackUp+" b_BackUp:"+nObjectB_BackUp+" c_BackUp:"+nObjectC_BackUp);
				Frame Object1=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiSet"));				
				
            	if (nToolMode==1){
            		Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiPolish"));
				}
				else if(nToolMode==2){
			    	Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanPolish"));
				}
				else if(nToolMode==3){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiSet"));
				}
				else if(nToolMode==4){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanSet"));
				}
				else if(nToolMode==5){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiPolish"));
				}
				else if(nToolMode==6){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanPolish"));
				}
				else if(nToolMode==7){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiSet"));
				}
				else if(nToolMode==8){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanSet"));
				}
				else if (nToolMode==9){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"));
				}
				else if(nToolMode==10){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanPolish"));
				}
				else if(nToolMode==11){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiSet"));
				}
				else if(nToolMode==12){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanSet"));
				}
				else if(nToolMode==13){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiPolish"));
				}
				else if(nToolMode==14){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanPolish"));
				}
				else if(nToolMode==15){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiSet"));
				}
				else if(nToolMode==16){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanSet"));
				}
				else if (nToolMode==17){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiPolish"));
				}
				else if(nToolMode==18){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanPolish"));
				}
				else if(nToolMode==19){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiSet"));
				}
				else if(nToolMode==20){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanSet"));
				}
				else if(nToolMode==21){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiPolish"));
				}
				else if(nToolMode==22){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanPolish"));
				}
				else if(nToolMode==23){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiSet"));
				}
				else if(nToolMode==24){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanSet"));
				}
				else if (nToolMode==25){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiPolish"));
				}
				else if(nToolMode==26){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanPolish"));
				}
				else if(nToolMode==27){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiSet"));
				}
				else if(nToolMode==28){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanSet"));
				}
				else if(nToolMode==29){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiPolish"));
				}
				else if(nToolMode==30){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanPolish"));
				}
				else if(nToolMode==31){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiSet"));
				}
				else if(nToolMode==32){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanSet"));
				}
				else if (nToolMode==33){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiPolish"));
				}
				else if(nToolMode==34){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanPolish"));
				}
				else if(nToolMode==35){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiSet"));
				}
				else if(nToolMode==36){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanSet"));
				}
				else if(nToolMode==37){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiPolish"));
				}
				else if(nToolMode==38){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanPolish"));
				}
				else if(nToolMode==39){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiSet"));
				}
				else if(nToolMode==40){
					Object=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanSet"));
				}
				
				//判断使用最优值还是备选值
				if(nObjectA>0){
					Object1.setAlphaRad(Math.toRadians(nObjectA));
					Object1.setBetaRad(Math.toRadians(nObjectB));
					Object1.setGammaRad(Math.toRadians(nObjectC));
				}
				else{
					Object1.setAlphaRad(Math.toRadians(nObjectA_BackUp));
					Object1.setBetaRad(Math.toRadians(nObjectB_BackUp));
					Object1.setGammaRad(Math.toRadians(nObjectC_BackUp));
				}

//				System.out.println("x"+Object1.getX()+"y"+Object1.getY()+"z"+Object1.getZ()+"a:"+Object1.getAlphaRad()+" b:"+Object1.getBetaRad()+" c:"+Object1.getGammaRad());
				
				Frame Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiSet"));
				
				if (nToolMode==1){
            		Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiPolish"));
				}
				else if(nToolMode==2){
			    	Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanPolish"));
				}
				else if(nToolMode==3){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiSet"));
				}
				else if(nToolMode==4){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanSet"));
				}
				else if(nToolMode==5){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiPolish"));
				}
				else if(nToolMode==6){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanPolish"));
				}
				else if(nToolMode==7){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiSet"));
				}
				else if(nToolMode==8){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanSet"));
				}
				else if (nToolMode==9){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"));
				}
				else if(nToolMode==10){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanPolish"));
				}
				else if(nToolMode==11){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiSet"));
				}
				else if(nToolMode==12){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanSet"));
				}
				else if(nToolMode==13){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiPolish"));
				}
				else if(nToolMode==14){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanPolish"));
				}
				else if(nToolMode==15){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiSet"));
				}
				else if(nToolMode==16){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanSet"));
				}
				else if (nToolMode==17){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiPolish"));
				}
				else if(nToolMode==18){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanPolish"));
				}
				else if(nToolMode==19){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiSet"));
				}
				else if(nToolMode==20){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanSet"));
				}
				else if(nToolMode==21){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiPolish"));
				}
				else if(nToolMode==22){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanPolish"));
				}
				else if(nToolMode==23){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiSet"));
				}
				else if(nToolMode==24){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanSet"));
				}
				else if (nToolMode==25){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiPolish"));
				}
				else if(nToolMode==26){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanPolish"));
				}
				else if(nToolMode==27){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiSet"));
				}
				else if(nToolMode==28){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanSet"));
				}
				else if(nToolMode==29){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiPolish"));
				}
				else if(nToolMode==30){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanPolish"));
				}
				else if(nToolMode==31){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiSet"));
				}
				else if(nToolMode==32){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanSet"));
				}
				else if (nToolMode==33){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiPolish"));
				}
				else if(nToolMode==34){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanPolish"));
				}
				else if(nToolMode==35){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiSet"));
				}
				else if(nToolMode==36){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanSet"));
				}
				else if(nToolMode==37){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiPolish"));
				}
				else if(nToolMode==38){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanPolish"));
				}
				else if(nToolMode==39){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiSet"));
				}
				else if(nToolMode==40){
					Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanSet"));
				}
				

				
				Frame Object4=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiSet"));
				if (nToolMode==1){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 60)));
					Object4 = Object4.transform((Transformation.ofDeg(-0.6  ,-125.8 ,-250.4, 0, 0, 0)));
//					System.out.println("Object4"+Object4);	
				}
				else if(nToolMode==2){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0 ,0, 0, 0, 0)));
//					System.out.println("Object4"+Object4);	
				}
				else if(nToolMode==3){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 60)));
					Object4 = Object4.transform((Transformation.ofDeg(-0.9 ,-168.5,-275.2, 0, 0, 0)));
//					System.out.println("Object4"+Object4);	
				}
				else if(nToolMode==4){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
//					System.out.println("Object4"+Object4);	
				}
				else if(nToolMode==5){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, -60)));
					Object4 = Object4.transform((Transformation.ofDeg(0.6 ,125.8,-250.4, 0, 0, 0)));
//					System.out.println("Object4"+Object4);	
				}
				else if(nToolMode==6){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
//					System.out.println("Object4"+Object4);	
				}
				else if(nToolMode==7){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, -60)));
					Object4 = Object4.transform((Transformation.ofDeg(0.9 ,168.5,-275.2, 0, 0, 0)));
//					System.out.println("Object4"+Object4);	
				}
				else if(nToolMode==8){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
//					System.out.println("Object4"+Object4);	
				}
				else if(nToolMode==9){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 60)));
					Object4 = Object4.transform((Transformation.ofDeg(0.4 ,-126.1,-249.6, 0, 0, 0)));
				}
				else if(nToolMode==10){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 60)));
					Object4 = Object4.transform((Transformation.ofDeg(2.8 ,-158.3,-308.8, 0, 0, 0)));
				}
				else if(nToolMode==11){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 60)));
					Object4 = Object4.transform((Transformation.ofDeg(0.6 ,-168.7,-274, 0, 0, 0)));
				}
				else if(nToolMode==12){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				else if(nToolMode==13){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, -60)));
					Object4 = Object4.transform((Transformation.ofDeg(-0.4 ,126.1,-249.6, 0, 0, 0)));
				}
				else if(nToolMode==14){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, -60)));
					Object4 = Object4.transform((Transformation.ofDeg(-2.8 ,158.3,308.8, 0, 0, 0)));
				}
				else if(nToolMode==15){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, -60)));
					Object4 = Object4.transform((Transformation.ofDeg(-0.6 ,168.7,-274, 0, 0, 0)));
				}
				else if(nToolMode==16){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				else if(nToolMode==17){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 60)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,-125.3,-257, 0, 0, 0)));
				}
				else if(nToolMode==18){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				else if(nToolMode==19){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 60)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,-168,-281.6, 0, 0, 0)));
				}
				else if(nToolMode==20){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				else if(nToolMode==21){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				else if(nToolMode==22){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				else if(nToolMode==23){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				else if(nToolMode==24){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				else if(nToolMode==25){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 60)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,-125.3,-257, 0, 0, 0)));
				}
				else if(nToolMode==26){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				else if(nToolMode==27){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 60)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,-168,-281.6, 0, 0, 0)));
				}
				else if(nToolMode==28){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				else if(nToolMode==29){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				else if(nToolMode==30){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				else if(nToolMode==31){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				else if(nToolMode==32){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				else if(nToolMode==33){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 60)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,-125.3,-257, 0, 0, 0)));
				}
				else if(nToolMode==34){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				else if(nToolMode==35){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 60)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,-168,-281.6, 0, 0, 0)));
				}
				else if(nToolMode==36){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				else if(nToolMode==37){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				else if(nToolMode==38){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				else if(nToolMode==39){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				else if(nToolMode==40){
					Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,0, 0, 0, 0)));
				}
				try{
					
					lbr.getInverseKinematicFromFrameAndRedundancy(Object4);
//					System.out.println("222");	
				Frame Object5 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiSet"));
//				System.out.println("333");	
				Object5.setX(Object4.getX());
				Object5.setY(Object4.getY());
				Object5.setZ(Object4.getZ());
				Object5.setAlphaRad(Object4.getAlphaRad());
				Object5.setBetaRad(Object4.getBetaRad());
				Object5.setGammaRad(Object4.getGammaRad());
//				System.out.println("Object5"+Object5);
				JointPosition test=lbr.getInverseKinematicFromFrameAndRedundancy(Object5);
				
				
				
				
				
				if (nToolMode==1){					
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();				
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiPolish"), cmdPos2);
					
					
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiPolish"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);  	    
				   DistanceToPlane.getGammaRad();
//			       	   System.out.println("AAAA:"+DistanceToPlane.getAlphaRad());
				   
				    
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				     Err="1,";
				  needle.getFrame("/zuo_21001_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }
				    else{
				    	needle.getFrame("/zuo_21001_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }					
				}
				else if(nToolMode==2){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanPolish"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));		
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanPolish"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);
				    DistanceToPlane.getGammaRad();  	    
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());					
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/zuo_21001_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				   	    }
				   	    else{
				   	    	needle.getFrame("/zuo_21001_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				   	    }	    
				}
				else if(nToolMode==3){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiSet"));
//						System.out.println("1:"+cmdPos);
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiSet"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					final CartesianImpedanceControlMode cartImp = ZoneLimit();
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiSet"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				    DistanceToPlane.getGammaRad();
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//			       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//			       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/zuo_21001_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				   	    else{
				   	    	needle.getFrame("/zuo_21001_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				    
				}
				else if(nToolMode==4){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanSet"));
//						System.out.println("1:"+cmdPos);
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanSet"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					final CartesianImpedanceControlMode cartImp = ZoneLimit();
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanSet"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				    DistanceToPlane.getGammaRad();
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//			       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//			       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/zuo_21001_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				   	    else{
				   	    	needle.getFrame("/zuo_21001_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				    
				}
				else if (nToolMode==5){					
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();				
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiPolish"), cmdPos2);
					
					
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiPolish"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);  	    
				   DistanceToPlane.getGammaRad();
//			       	   System.out.println("AAAA:"+DistanceToPlane.getAlphaRad());
				   
				    
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				     Err="1,";
				  needle.getFrame("/you_21001_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }
				    else{
				    	needle.getFrame("/you_21001_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }					
				}
				else if(nToolMode==6){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanPolish"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));		
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanPolish"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);
				    DistanceToPlane.getGammaRad();  	    
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());					
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/you_21001_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				   	    }
				   	    else{
				   	    	needle.getFrame("/you_21001_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				   	    }	    
				}
				else if(nToolMode==7){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiSet"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiSet"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					final CartesianImpedanceControlMode cartImp = ZoneLimit();
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiSet"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				    DistanceToPlane.getGammaRad();
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//			       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//			       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/you_21001_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				   	    else{
				   	    	needle.getFrame("/you_21001_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				    
				}
				else if(nToolMode==8){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanSet"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanSet"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					final CartesianImpedanceControlMode cartImp = ZoneLimit();
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanSet"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				    DistanceToPlane.getGammaRad();
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//			       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//			       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/you_21001_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				   	    else{
				   	    	needle.getFrame("/you_21001_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				    
				}
				else if (nToolMode==9){					
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();				
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"), cmdPos2);
					
					
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);  	    
				   DistanceToPlane.getGammaRad();
//			       	   System.out.println("AAAA:"+DistanceToPlane.getAlphaRad());
				   
				    
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				     Err="1,";
				  needle.getFrame("/zuo_21002_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }
				    else{
				    	needle.getFrame("/zuo_21002_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }					
				}
				else if(nToolMode==10){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanPolish"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));		
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanPolish"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);
				    DistanceToPlane.getGammaRad();  	    
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());					
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/zuo_21002_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				   	    }
				   	    else{
				   	    	needle.getFrame("/zuo_21002_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				   	    }	    
				}
				else if(nToolMode==11){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiSet"));
//						System.out.println("1:"+cmdPos);
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiSet"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					final CartesianImpedanceControlMode cartImp = ZoneLimit();
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiSet"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				    DistanceToPlane.getGammaRad();
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//			       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//			       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/zuo_21002_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				   	    else{
				   	    	needle.getFrame("/zuo_21002_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				    
				}
				else if(nToolMode==12){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanSet"));
//						System.out.println("1:"+cmdPos);
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanSet"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					final CartesianImpedanceControlMode cartImp = ZoneLimit();
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanSet"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				    DistanceToPlane.getGammaRad();
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//			       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//			       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/zuo_21002_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				   	    else{
				   	    	needle.getFrame("/zuo_21002_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				    
				}
				else if (nToolMode==13){					
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();				
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiPolish"), cmdPos2);
					
					
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiPolish"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);  	    
				   DistanceToPlane.getGammaRad();
//			       	   System.out.println("AAAA:"+DistanceToPlane.getAlphaRad());
				   
				    
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				     Err="1,";
				  needle.getFrame("/you_21002_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }
				    else{
				    	needle.getFrame("/you_21002_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }					
				}
				else if(nToolMode==14){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanPolish"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));		
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanPolish"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);
				    DistanceToPlane.getGammaRad();  	    
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());					
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/you_21002_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				   	    }
				   	    else{
				   	    	needle.getFrame("/you_21002_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				   	    }	    
				}
				else if(nToolMode==15){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiSet"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiSet"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					final CartesianImpedanceControlMode cartImp = ZoneLimit();
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiSet"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				    DistanceToPlane.getGammaRad();
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//			       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//			       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/you_21002_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				   	    else{
				   	    	needle.getFrame("/you_21002_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				    
				}
				else if(nToolMode==16){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanSet"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanSet"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					final CartesianImpedanceControlMode cartImp = ZoneLimit();
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanSet"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				    DistanceToPlane.getGammaRad();
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//			       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//			       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/you_21002_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				   	    else{
				   	    	needle.getFrame("/you_21002_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				    
				}	
				else if (nToolMode==17){					
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();				
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiPolish"), cmdPos2);
					
					
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiPolish"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);  	    
				   DistanceToPlane.getGammaRad();
//			       	   System.out.println("AAAA:"+DistanceToPlane.getAlphaRad());
				   
				    
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				     Err="1,";
				  needle.getFrame("/zuo_21003_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }
				    else{
				    	needle.getFrame("/zuo_21003_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }					
				}
				else if(nToolMode==18){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanPolish"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));		
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanPolish"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);
				    DistanceToPlane.getGammaRad();  	    
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());					
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/zuo_21003_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				   	    }
				   	    else{
				   	    	needle.getFrame("/zuo_21003_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				   	    }	    
				}
				else if(nToolMode==19){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiSet"));
//						System.out.println("1:"+cmdPos);
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiSet"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					final CartesianImpedanceControlMode cartImp = ZoneLimit();
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiSet"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				    DistanceToPlane.getGammaRad();
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//			       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//			       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/zuo_21003_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				   	    else{
				   	    	needle.getFrame("/zuo_21003_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				    
				}
				else if(nToolMode==20){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanSet"));
//						System.out.println("1:"+cmdPos);
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanSet"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					final CartesianImpedanceControlMode cartImp = ZoneLimit();
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanSet"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				    DistanceToPlane.getGammaRad();
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//			       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//			       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/zuo_21003_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				   	    else{
				   	    	needle.getFrame("/zuo_21003_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				    
				}
				else if (nToolMode==21){					
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();				
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiPolish"), cmdPos2);
					
					
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiPolish"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);  	    
				   DistanceToPlane.getGammaRad();
//			       	   System.out.println("AAAA:"+DistanceToPlane.getAlphaRad());
				   
				    
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				     Err="1,";
				  needle.getFrame("/you_21003_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }
				    else{
				    	needle.getFrame("/you_21003_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }					
				}
				else if(nToolMode==22){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanPolish"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));		
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanPolish"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);
				    DistanceToPlane.getGammaRad();  	    
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());					
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/you_21003_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				   	    }
				   	    else{
				   	    	needle.getFrame("/you_21003_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				   	    }	    
				}
				else if(nToolMode==23){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiSet"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiSet"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					final CartesianImpedanceControlMode cartImp = ZoneLimit();
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiSet"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				    DistanceToPlane.getGammaRad();
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//			       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//			       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/you_21003_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				   	    else{
				   	    	needle.getFrame("/you_21003_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				    
				}
				else if(nToolMode==24){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanSet"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanSet"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					final CartesianImpedanceControlMode cartImp = ZoneLimit();
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanSet"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				    DistanceToPlane.getGammaRad();
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//			       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//			       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/you_21003_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				   	    else{
				   	    	needle.getFrame("/you_21003_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				    
				}	
				else if (nToolMode==25){					
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();				
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiPolish"), cmdPos2);
					
					
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiPolish"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);  	    
				   DistanceToPlane.getGammaRad();
//			       	   System.out.println("AAAA:"+DistanceToPlane.getAlphaRad());
				   
				    
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				     Err="1,";
				  needle.getFrame("/zuo_21004_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }
				    else{
				    	needle.getFrame("/zuo_21004_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }					
				}
				else if(nToolMode==26){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanPolish"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));		
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanPolish"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);
				    DistanceToPlane.getGammaRad();  	    
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());					
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/zuo_21004_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				   	    }
				   	    else{
				   	    	needle.getFrame("/zuo_21004_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				   	    }	    
				}
				else if(nToolMode==27){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiSet"));
//						System.out.println("1:"+cmdPos);
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiSet"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					final CartesianImpedanceControlMode cartImp = ZoneLimit();
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiSet"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				    DistanceToPlane.getGammaRad();
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//			       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//			       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/zuo_21004_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				   	    else{
				   	    	needle.getFrame("/zuo_21004_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				    
				}
				else if(nToolMode==28){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanSet"));
//						System.out.println("1:"+cmdPos);
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanSet"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					final CartesianImpedanceControlMode cartImp = ZoneLimit();
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanSet"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				    DistanceToPlane.getGammaRad();
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//			       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//			       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/zuo_21004_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				   	    else{
				   	    	needle.getFrame("/zuo_21004_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				    
				}
				else if (nToolMode==29){					
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();				
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiPolish"), cmdPos2);
					
					
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiPolish"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);  	    
				   DistanceToPlane.getGammaRad();
//			       	   System.out.println("AAAA:"+DistanceToPlane.getAlphaRad());
				   
				    
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				     Err="1,";
				  needle.getFrame("/you_21004_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }
				    else{
				    	needle.getFrame("/you_21004_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }					
				}
				else if(nToolMode==30){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanPolish"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanPolish"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));		
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanPolish"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);
				    DistanceToPlane.getGammaRad();  	    
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());					
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/you_21004_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				   	    }
				   	    else{
				   	    	needle.getFrame("/you_21004_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				   	    }	    
				}
				else if(nToolMode==31){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiSet"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiSet"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					final CartesianImpedanceControlMode cartImp = ZoneLimit();
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiSet"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				    DistanceToPlane.getGammaRad();
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//			       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//			       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/you_21004_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				   	    else{
				   	    	needle.getFrame("/you_21004_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				    
				}
				else if(nToolMode==32){
					Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanSet"));
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanSet"), cmdPos2);
					pre_Point.setX(cmdPos.getX());
					pre_Point.setY(cmdPos.getY());
					pre_Point.setZ(cmdPos.getZ());
					pre_Point.setAlphaRad(Object1.getAlphaRad());
					pre_Point.setBetaRad(Object1.getBetaRad());
					pre_Point.setGammaRad(Object1.getGammaRad());
					//用优化角度还是直接角度赋值
//						pre_Point.setAlphaRad(Math.toRadians(nA));
//						pre_Point.setBetaRad(Math.toRadians(nB));
//						pre_Point.setGammaRad(Math.toRadians(nC));
					final CartesianImpedanceControlMode cartImp = ZoneLimit();
					
					//判断
					Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanSet"));
				    Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				    DistanceToPlane.getGammaRad();
				    System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//			       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//			       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				    if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				   	     Err="1,";
				   	     needle.getFrame("/you_21004_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				   	    else{
				   	    	needle.getFrame("/you_21004_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				   	    }
				    
				}	
							
					else if (nToolMode==33){					
				Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();				
				Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiPolish"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiPolish"), cmdPos2);
				
				
				pre_Point.setX(cmdPos.getX());
				pre_Point.setY(cmdPos.getY());
				pre_Point.setZ(cmdPos.getZ());
				pre_Point.setAlphaRad(Object1.getAlphaRad());
				pre_Point.setBetaRad(Object1.getBetaRad());
				pre_Point.setGammaRad(Object1.getGammaRad());
				//用优化角度还是直接角度赋值
//					pre_Point.setAlphaRad(Math.toRadians(nA));
//					pre_Point.setBetaRad(Math.toRadians(nB));
//					pre_Point.setGammaRad(Math.toRadians(nC));
				
				//判断
				Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiPolish"));
				Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);  	    
     	   DistanceToPlane.getGammaRad();
//		       	   System.out.println("AAAA:"+DistanceToPlane.getAlphaRad());
     	   
				
				if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				 Err="1,";
     	  needle.getFrame("/zuo_21005_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				}
				else{
					needle.getFrame("/zuo_21005_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				}					
					}
						else if(nToolMode==34){
				Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
				Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanPolish"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanPolish"), cmdPos2);
				pre_Point.setX(cmdPos.getX());
				pre_Point.setY(cmdPos.getY());
				pre_Point.setZ(cmdPos.getZ());
				pre_Point.setAlphaRad(Object1.getAlphaRad());
				pre_Point.setBetaRad(Object1.getBetaRad());
				pre_Point.setGammaRad(Object1.getGammaRad());
				//用优化角度还是直接角度赋值
//					pre_Point.setAlphaRad(Math.toRadians(nA));
//					pre_Point.setBetaRad(Math.toRadians(nB));
//					pre_Point.setGammaRad(Math.toRadians(nC));		
				//判断
				Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanPolish"));
				Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);
				DistanceToPlane.getGammaRad();  	    
				System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());					
				if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				     Err="1,";
				     needle.getFrame("/zuo_21005_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }
				    else{
				    	needle.getFrame("/zuo_21005_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }	    
						}
						else if(nToolMode==35){
				Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
				Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiSet"));
//					System.out.println("1:"+cmdPos);
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiSet"), cmdPos2);
				pre_Point.setX(cmdPos.getX());
				pre_Point.setY(cmdPos.getY());
				pre_Point.setZ(cmdPos.getZ());
				pre_Point.setAlphaRad(Object1.getAlphaRad());
				pre_Point.setBetaRad(Object1.getBetaRad());
				pre_Point.setGammaRad(Object1.getGammaRad());
				//用优化角度还是直接角度赋值
//					pre_Point.setAlphaRad(Math.toRadians(nA));
//					pre_Point.setBetaRad(Math.toRadians(nB));
//					pre_Point.setGammaRad(Math.toRadians(nC));
				final CartesianImpedanceControlMode cartImp = ZoneLimit();
				
				//判断
				Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiSet"));
				Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				DistanceToPlane.getGammaRad();
				System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//		       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//		       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				     Err="1,";
				     needle.getFrame("/zuo_21005_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				    }
				    else{
				    	needle.getFrame("/zuo_21005_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				    }
				
}
else if(nToolMode==36){
				Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
				Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanSet"));
//					System.out.println("1:"+cmdPos);
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanSet"), cmdPos2);
				pre_Point.setX(cmdPos.getX());
				pre_Point.setY(cmdPos.getY());
				pre_Point.setZ(cmdPos.getZ());
				pre_Point.setAlphaRad(Object1.getAlphaRad());
				pre_Point.setBetaRad(Object1.getBetaRad());
				pre_Point.setGammaRad(Object1.getGammaRad());
				//用优化角度还是直接角度赋值
//					pre_Point.setAlphaRad(Math.toRadians(nA));
//					pre_Point.setBetaRad(Math.toRadians(nB));
//					pre_Point.setGammaRad(Math.toRadians(nC));
				final CartesianImpedanceControlMode cartImp = ZoneLimit();
				
				//判断
				Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanSet"));
				Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				DistanceToPlane.getGammaRad();
				System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//		       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//		       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				     Err="1,";
				     needle.getFrame("/zuo_21005_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				    }
				    else{
				    	needle.getFrame("/zuo_21005_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				    }
				
}
else if (nToolMode==37){					
				Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();				
				Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiPolish"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiPolish"), cmdPos2);
				
				
				pre_Point.setX(cmdPos.getX());
				pre_Point.setY(cmdPos.getY());
				pre_Point.setZ(cmdPos.getZ());
				pre_Point.setAlphaRad(Object1.getAlphaRad());
				pre_Point.setBetaRad(Object1.getBetaRad());
				pre_Point.setGammaRad(Object1.getGammaRad());
				//用优化角度还是直接角度赋值
//					pre_Point.setAlphaRad(Math.toRadians(nA));
//					pre_Point.setBetaRad(Math.toRadians(nB));
//					pre_Point.setGammaRad(Math.toRadians(nC));
				
				//判断
				Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiPolish"));
				Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);  	    
     	   DistanceToPlane.getGammaRad();
//		       	   System.out.println("AAAA:"+DistanceToPlane.getAlphaRad());
     	   
				
				if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				 Err="1,";
     	  needle.getFrame("/you_21005_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				}
				else{
					needle.getFrame("/you_21005_zhiPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				}					
}
else if(nToolMode==38){
				Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
				Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanPolish"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanPolish"), cmdPos2);
				pre_Point.setX(cmdPos.getX());
				pre_Point.setY(cmdPos.getY());
				pre_Point.setZ(cmdPos.getZ());
				pre_Point.setAlphaRad(Object1.getAlphaRad());
				pre_Point.setBetaRad(Object1.getBetaRad());
				pre_Point.setGammaRad(Object1.getGammaRad());
				//用优化角度还是直接角度赋值
//					pre_Point.setAlphaRad(Math.toRadians(nA));
//					pre_Point.setBetaRad(Math.toRadians(nB));
//					pre_Point.setGammaRad(Math.toRadians(nC));		
				//判断
				Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanPolish"));
				Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe);
				DistanceToPlane.getGammaRad();  	    
				System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());					
				if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				     Err="1,";
				     needle.getFrame("/you_21005_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }
				    else{
				    	needle.getFrame("/you_21005_wanPolish").move(lin(pre_Point).setJointVelocityRel(0.2));
				    }	    
}
else if(nToolMode==39){
				Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
				Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiSet"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiSet"), cmdPos2);
				pre_Point.setX(cmdPos.getX());
				pre_Point.setY(cmdPos.getY());
				pre_Point.setZ(cmdPos.getZ());
				pre_Point.setAlphaRad(Object1.getAlphaRad());
				pre_Point.setBetaRad(Object1.getBetaRad());
				pre_Point.setGammaRad(Object1.getGammaRad());
				//用优化角度还是直接角度赋值
//					pre_Point.setAlphaRad(Math.toRadians(nA));
//					pre_Point.setBetaRad(Math.toRadians(nB));
//					pre_Point.setGammaRad(Math.toRadians(nC));
				final CartesianImpedanceControlMode cartImp = ZoneLimit();
				
				//判断
				Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiSet"));
				Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				DistanceToPlane.getGammaRad();
				System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//		       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//		       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				     Err="1,";
				     needle.getFrame("/you_21005_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				    }
				    else{
				    	needle.getFrame("/you_21005_zhiSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				    }
				
}
else if(nToolMode==40){
				Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
				Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanSet"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanSet"), cmdPos2);
				pre_Point.setX(cmdPos.getX());
				pre_Point.setY(cmdPos.getY());
				pre_Point.setZ(cmdPos.getZ());
				pre_Point.setAlphaRad(Object1.getAlphaRad());
				pre_Point.setBetaRad(Object1.getBetaRad());
				pre_Point.setGammaRad(Object1.getGammaRad());
				//用优化角度还是直接角度赋值
//					pre_Point.setAlphaRad(Math.toRadians(nA));
//					pre_Point.setBetaRad(Math.toRadians(nB));
//					pre_Point.setGammaRad(Math.toRadians(nC));
				final CartesianImpedanceControlMode cartImp = ZoneLimit();
				
				//判断
				Frame cmdPosSafe = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanSet"));
				Transformation DistanceToPlane=pre_Point.staticTransformationTo(cmdPosSafe); 
				DistanceToPlane.getGammaRad();
				System.out.println("DistanceToPlane:"+DistanceToPlane.getAlphaRad());
//		       	    System.out.println("BBBB:"+DistanceToPlane.getBetaRad());
//		       	    System.out.println("CCCC:"+DistanceToPlane.getGammaRad());
				if(Math.abs(Math.toDegrees(DistanceToPlane.getAlphaRad()))>10){
				     Err="1,";
				     needle.getFrame("/you_21005_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				    }
				    else{
				    	needle.getFrame("/you_21005_wanSet").move(lin(pre_Point).setJointVelocityRel(0.2).setMode(cartImp));
				    }
				
}
				if(Math.toDegrees(test.get(JointEnum.J1))>11 || Math.toDegrees(test.get(JointEnum.J1))<-11){
//					System.out.println("err1："+Math.toDegrees(test.get(JointEnum.J1)));
				}
				if(Math.toDegrees(test.get(JointEnum.J2))>31 || Math.toDegrees(test.get(JointEnum.J2))<-15){
//					System.out.println("err2："+Math.toDegrees(test.get(JointEnum.J2)));
				}
				if(Math.toDegrees(test.get(JointEnum.J3))>46 || Math.toDegrees(test.get(JointEnum.J3))<-46){
//					System.out.println("err3："+Math.toDegrees(test.get(JointEnum.J3)));
				}
				if(Math.toDegrees(test.get(JointEnum.J4))>116 || Math.toDegrees(test.get(JointEnum.J4))<-1){
//					System.out.println("err4："+Math.toDegrees(test.get(JointEnum.J4)));
				}
				if(Math.toDegrees(test.get(JointEnum.J5))>66 || Math.toDegrees(test.get(JointEnum.J5))<-66){
//					System.out.println("err5："+Math.toDegrees(test.get(JointEnum.J5)));
				}
				if(Math.toDegrees(test.get(JointEnum.J6))>-46 || Math.toDegrees(test.get(JointEnum.J6))<-111){
//					System.out.println("err6："+Math.toDegrees(test.get(JointEnum.J6)));
				}
				if(Math.toDegrees(test.get(JointEnum.J7))>166 || Math.toDegrees(test.get(JointEnum.J7))<-166){
//					System.out.println("err7："+Math.toDegrees(test.get(JointEnum.J7)));
				}
//				System.out.println("J1："+Math.toDegrees(test.get(JointEnum.J1))+"   J2:"+Math.toDegrees(test.get(JointEnum.J2))+"   J3:"+Math.toDegrees(test.get(JointEnum.J3))+"   J4:"+Math.toDegrees(test.get(JointEnum.J4))+"   J5:"+Math.toDegrees(test.get(JointEnum.J5))+"   J6:"+Math.toDegrees(test.get(JointEnum.J6))+"   J7:"+Math.toDegrees(test.get(JointEnum.J7)) );
				}
				catch(Throwable cause)
				{
					System.out.println("OutOfRange2");
				}
						
				nWorkingmode=0;

				
			}
			
			else if(nWorkingmode==6){
				nLastWorkingmode=nWorkingmode;
//		        // Initialize Joint impedance mode    
//				System.out.println("JointimplentMode"+nWorkingmode);
//				moveToInitialPosition();
//		        final JointImpedanceControlMode jointImp = createJointImp();
//		        runSmartServoMotion(jointImp);
				
//				Frame Ptest1= getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();		
//				needle.getFrame("/zuo_21002_zhiPolish").move(ptp(Ptest1).setBlendingCart(0).setJointVelocityRel(0.2).setBlendingRel(0).setBlendingRel(0));
//				System.out.println("automode"+nWorkingmode);
//				Frame Ptest1= getApplicationData().getFrame("/P1").copyWithRedundancy();
				
				
				Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiSet"));

               //testdata x:735  y:7.59  z:122 Aï¼š-91 Bï¼š-40 Cï¼š-178 $cmd,ml,715,7,122,-91,-40,-178$
				//$cmd,RobotMove,1$
				if(nX!=0 && nY!=0 && nZ!=0){
					Ptest1.setX(nX);
					Ptest1.setY(nY);
					Ptest1.setZ(nZ);
//		     		System.out.println("nx:"+nX+"  ny:"+nY+"  nz:"+nZ+"  a:"+nZ);
					Ptest1.setAlphaRad(Math.toRadians(nA));
					Ptest1.setBetaRad(Math.toRadians(nB));
					Ptest1.setGammaRad(Math.toRadians(nC));

					
					if(Math.abs(nX)<2000 && Math.abs(nY)<2000 && Math.abs(nZ)<2000 && Math.abs(nA)<2000 && Math.abs(nB)<2000 && Math.abs(nC)<2000){
						//System.out.println("pre_Place11***:"+pre_Place);
						needle.getFrame("/zuo_21002_zhiSet").move(ptp(pre_Place).setJointVelocityRel(0.1));	
					}
					else{
						//System.out.println("Err_DangerPlace: "+"nX:"+nX+"nY:"+nY+"nZ:"+nZ+"nA:"+nA+"nB:"+nB+"nC:"+nC);
					}
					
					
//					ThreadUtil.milliSleep(500);
//					System.out.println("222");
				}
				else{
					ThreadUtil.milliSleep(10);
				}
				//nWorkingmode=0;
				
			}
			else if(nWorkingmode==7){
				nLastWorkingmode=nWorkingmode;
		        // Initialize Cartesian impedance mode    
//				System.out.println("CartesianimplentMode"+nWorkingmode);
//				moveToInitialPosition();
//				final CartesianImpedanceControlMode cartImp = createCartImp();
//		        runSmartCartesianMotion(cartImp);
           
		    	//空间打磨模式
//		    	final CartesianImpedanceControlMode cartImp = ConeLimit();	
		    	final CartesianImpedanceControlMode cartImp = ZoneLimit();
		    	System.out.println("ZoneLimit");
		    	JointPosition jReady =lbr.getCurrentJointPosition();
		    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/test"));
		    	needle.getFrame("/test").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
               
                

		        nWorkingmode=0;
			}
			else if(nWorkingmode==8){
				nLastWorkingmode=nWorkingmode;
				final CartesianImpedanceControlMode carthard = InstallLimit();
				System.out.println("InstallLimit");
				JointPosition jReady =lbr.getCurrentJointPosition();
				if(nToolMode==1){
					   System.out.println("nToolMode=1");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiPolish"));
				       needle.getFrame("//zuo_21001_zhiPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==2){
					   System.out.println("nToolMode=2");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanPolish"));
				       needle.getFrame("//zuo_21001_wanPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==3){
				   System.out.println("nToolMode=3");
			       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_zhiSet"));
			       needle.getFrame("/zuo_21001_zhiSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
				   System.out.println("movecomPds");
				   nWorkingmode=0;
				
				}
				else if(nToolMode==4){
					   System.out.println("nToolMode=4");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21001_wanSet"));
				       needle.getFrame("/zuo_21001_wanSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==5){
					   System.out.println("nToolMode=5");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiPolish"));
				       needle.getFrame("/you_21001_zhiPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==6){
					   System.out.println("nToolMode=6");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanPolish"));
				       needle.getFrame("/you_21001_wanPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==7){
					   System.out.println("nToolMode=7");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_zhiSet"));
				       needle.getFrame("/you_21001_zhiSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==8){
					   System.out.println("nToolMode=8");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001_wanSet"));
				       needle.getFrame("/you_21001_wanSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==9){
					   System.out.println("nToolMode=9");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"));
				       needle.getFrame("/zuo_21002_zhiPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==10){
					   System.out.println("nToolMode=10");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanPolish"));
				       needle.getFrame("/zuo_21002_wanPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==11){
					   System.out.println("nToolMode=11");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiSet"));
				       needle.getFrame("/zuo_21002_zhiSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==12){
					   System.out.println("nToolMode=12");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_wanSet"));
				       needle.getFrame("/zuo_21002_wanSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==13){
					   System.out.println("nToolMode=13");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiPolish"));
				       needle.getFrame("/you_21002_zhiPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==14){
					   System.out.println("nToolMode=14");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanPolish"));
				       needle.getFrame("/you_21002_wanPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==15){
					   System.out.println("nToolMode=15");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_zhiSet"));
				       needle.getFrame("/you_21002_zhiSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==16){
					   System.out.println("nToolMode=16");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21002_wanSet"));
				       needle.getFrame("/you_21002_wanSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				
				else if(nToolMode==17){
					   System.out.println("nToolMode=17");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiPolish"));
				       needle.getFrame("/zuo_21003_zhiPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==18){
					   System.out.println("nToolMode=18");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanPolish"));
				       needle.getFrame("/zuo_21003_wanPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==19){
					   System.out.println("nToolMode=19");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_zhiSet"));
				       needle.getFrame("/zuo_21003_zhiSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==20){
					   System.out.println("nToolMode=20");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21003_wanSet"));
				       needle.getFrame("/zuo_21003_wanSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==21){
					   System.out.println("nToolMode=21");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiPolish"));
				       needle.getFrame("/you_21003_zhiPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==22){
					   System.out.println("nToolMode=22");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanPolish"));
				       needle.getFrame("/you_21003_wanPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==23){
					   System.out.println("nToolMode=23");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_zhiSet"));
				       needle.getFrame("/you_21003_zhiSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==24){
					   System.out.println("nToolMode=24");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21003_wanSet"));
				       needle.getFrame("/you_21003_wanSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==25){
					   System.out.println("nToolMode=25");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiPolish"));
				       needle.getFrame("/zuo_21004_zhiPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==26){
					   System.out.println("nToolMode=26");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanPolish"));
				       needle.getFrame("/zuo_21004_wanPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==27){
					   System.out.println("nToolMode=27");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_zhiSet"));
				       needle.getFrame("/zuo_21004_zhiSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==28){
					   System.out.println("nToolMode=28");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21004_wanSet"));
				       needle.getFrame("/zuo_21004_wanSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==29){
					   System.out.println("nToolMode=29");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiPolish"));
				       needle.getFrame("/you_21004_zhiPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==30){
					   System.out.println("nToolMode=30");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanPolish"));
				       needle.getFrame("/you_21004_wanPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==31){
					   System.out.println("nToolMode=31");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_zhiSet"));
				       needle.getFrame("/you_21004_zhiSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==32){
					   System.out.println("nToolMode=32");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21004_wanSet"));
				       needle.getFrame("/you_21004_wanSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				
				else if(nToolMode==33){
					   System.out.println("nToolMode=33");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiPolish"));
				       needle.getFrame("/zuo_21005_zhiPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==34){
					   System.out.println("nToolMode=34");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanPolish"));
				       needle.getFrame("/zuo_21005_wanPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==35){
					   System.out.println("nToolMode=35");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_zhiSet"));
				       needle.getFrame("/zuo_21005_zhiSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==36){
					   System.out.println("nToolMode=36");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21005_wanSet"));
				       needle.getFrame("/zuo_21005_wanSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==37){
					   System.out.println("nToolMode=37");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiPolish"));
				       needle.getFrame("/you_21005_zhiPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==38){
					   System.out.println("nToolMode=38");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanPolish"));
				       needle.getFrame("/you_21005_wanPolish").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==39){
					   System.out.println("nToolMode=39");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_zhiSet"));
				       needle.getFrame("/you_21005_zhiSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				else if(nToolMode==40){
					   System.out.println("nToolMode=40");
				       Frame comPds = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21005_wanSet"));
				       needle.getFrame("/you_21005_wanSet").move(ptp(comPds).setJointVelocityRel(0.1).setMode(carthard));
					   System.out.println("movecomPds");
					   nWorkingmode=0;
					
					}
				
				nWorkingmode=0;
			}
			else if(nWorkingmode==9){
				System.out.println("nWorkingmode=9");
				//pingyi
				if (nTest_rotation==0) {
					jointPos_register =new JointPosition(   Math.toRadians(11.10),
							Math.toRadians(1.64),
							Math.toRadians(-13.80),
							Math.toRadians(60.85),
							Math.toRadians(-62.84),
							Math.toRadians(-96.21),
							Math.toRadians(155.48));	
				}
				else if (nTest_rotation==1) {
					jointPos_register =new JointPosition(   Math.toRadians(3.67),
							Math.toRadians(-24.06),
							Math.toRadians(-13.80),
							Math.toRadians(26.46),
							Math.toRadians(-55.50),
							Math.toRadians(-99.45),
							Math.toRadians(146.78));	
				}
				else if (nTest_rotation==2) {
					jointPos_register =new JointPosition(   Math.toRadians(-20.79),
							Math.toRadians(-16.63),
							Math.toRadians(-13.79),
							Math.toRadians(37.41),
							Math.toRadians(-41.13),
							Math.toRadians(-80.24),
							Math.toRadians(137.37));	
				}
				else if (nTest_rotation==3) {
					jointPos_register =new JointPosition(   Math.toRadians(-0.57),
							Math.toRadians(-2.25),
							Math.toRadians(-13.79),
							Math.toRadians(77.84),
							Math.toRadians(-58.62),
							Math.toRadians(-76.71),
							Math.toRadians(168.27));	
				}
				else if (nTest_rotation==4) {
					jointPos_register =new JointPosition(   Math.toRadians(7.26),
							Math.toRadians(-21.97),
							Math.toRadians(-13.79),
							Math.toRadians(36.89),
							Math.toRadians(-56.71),
							Math.toRadians(-96.99),
							Math.toRadians(154.94));
				}
				else if (nTest_rotation==5) {
					jointPos_register =new JointPosition(   Math.toRadians(24.39),
							Math.toRadians(-19.64),
							Math.toRadians(-13.79),
							Math.toRadians(48.61),
							Math.toRadians(-61.97),
							Math.toRadians(-105.73),
							Math.toRadians(169.57));
				}
				else if (nTest_rotation==6) {
					jointPos_register =new JointPosition(   Math.toRadians(0.51),
							Math.toRadians(-7.20),
							Math.toRadians(-13.79),
							Math.toRadians(79.14),
							Math.toRadians(-59.10),
							Math.toRadians(-74.66),
							Math.toRadians(174.37));
				}
				else if (nTest_rotation==7) {
					jointPos_register =new JointPosition(   Math.toRadians(24.61),
							Math.toRadians(16.88),
							Math.toRadians(-13.79),
							Math.toRadians(91.12),
							Math.toRadians(-68.90),
							Math.toRadians(-100.27),
							Math.toRadians(174.89));
				}
				else if (nTest_rotation==8) {
					jointPos_register =new JointPosition(   Math.toRadians(16.00),
							Math.toRadians(8.06),
							Math.toRadians(-13.79),
							Math.toRadians(69.72),
							Math.toRadians(-66.10),
							Math.toRadians(-98.24),
							Math.toRadians(159.53));
				}
				else if (nTest_rotation==9) {
					jointPos_register =new JointPosition(   Math.toRadians(12.50),
							Math.toRadians(-3.31),
							Math.toRadians(-13.79),
							Math.toRadians(72.74),
							Math.toRadians(-61.42),
							Math.toRadians(-89.92),
							Math.toRadians(171.08));
				}
				else if (nTest_rotation==10) {
					jointPos_register =new JointPosition(   Math.toRadians(13.01),
							Math.toRadians(-17.57),
							Math.toRadians(-13.79),
							Math.toRadians(39.37),
							Math.toRadians(-60.38),
							Math.toRadians(-101.54),
							Math.toRadians(155.10));
				}
				else if (nTest_rotation==11) {
					jointPos_register =new JointPosition(   Math.toRadians(9.69),
							Math.toRadians(8.34),
							Math.toRadians(-13.79),
							Math.toRadians(84.10),
							Math.toRadians(-63.39),
							Math.toRadians(-86.97),
							Math.toRadians(169.55));
				}
				else if (nTest_rotation==12) {
					jointPos_register =new JointPosition(   Math.toRadians(-13.11),
							Math.toRadians(-5.52),
							Math.toRadians(-13.80),
							Math.toRadians(81.46),
							Math.toRadians(-56.11),
							Math.toRadians(-62.50),
							Math.toRadians(167.46));
				}
				else if (nTest_rotation==13) {
					jointPos_register =new JointPosition(   Math.toRadians(25.68),
							Math.toRadians(3.82),
							Math.toRadians(-13.80),
							Math.toRadians(76.22),
							Math.toRadians(-66.43),
							Math.toRadians(-102.35),
							Math.toRadians(173.40));
				}
				else if (nTest_rotation==14) {
					jointPos_register =new JointPosition(   Math.toRadians(0.88),
							Math.toRadians(3.64),
							Math.toRadians(-13.79),
							Math.toRadians(87.82),
							Math.toRadians(-61.28),
							Math.toRadians(-75.58),
							Math.toRadians(-172.74));
				}
				///
				
				
				

				 Err="10,";
					lbr.move(ptp(jointPos_register).setJointVelocityRel(0.2));
					Err="0,";
					nTest_rotation++;
					if (nTest_rotation==15){
						nTest_rotation=0;
					}
					nWorkingmode=0;
			}
			
			else if(nWorkingmode==10){
				System.out.println("nWorkingmode=10");
				//xuanzhuan 
				if (nTest_rotation==0) {
					jointPos_register =new JointPosition(   Math.toRadians(-32.44),
							Math.toRadians(16.25),
							Math.toRadians(20.17),
							Math.toRadians(91.63),
							Math.toRadians(-31.09),
							Math.toRadians(-41.86),
							Math.toRadians(123.74));	
				}
				else if (nTest_rotation==1) {
					jointPos_register =new JointPosition(   Math.toRadians(-4.58),
							Math.toRadians(-10.32),
							Math.toRadians(20.16),
							Math.toRadians(66.95),
							Math.toRadians(-72.82),
							Math.toRadians(-96.57),
							Math.toRadians(174.43));	
				}
				else if (nTest_rotation==2) {
					jointPos_register =new JointPosition(   Math.toRadians(-45.19),
							Math.toRadians(28.53),
							Math.toRadians(20.16),
							Math.toRadians(109.96),
							Math.toRadians(-33.90),
							Math.toRadians(-35.02),
							Math.toRadians(158.77));	
				}
				else if (nTest_rotation==3) {
					jointPos_register =new JointPosition(   Math.toRadians(-15.34),
							Math.toRadians(-9.46),
							Math.toRadians(20.16),
							Math.toRadians(68.25),
							Math.toRadians(-66.70),
							Math.toRadians(-68.23),
							Math.toRadians(140.27));	
				}
				else if (nTest_rotation==4) {
					jointPos_register =new JointPosition(   Math.toRadians(-20.55),
							Math.toRadians(-14.36),
							Math.toRadians(20.17),
							Math.toRadians(58.99),
							Math.toRadians(-70.07),
							Math.toRadians(-84.24),
							Math.toRadians(165.02));
				}
				else if (nTest_rotation==5) {
					jointPos_register =new JointPosition(   Math.toRadians(-30.02),
							Math.toRadians(-7.99),
							Math.toRadians(20.16),
							Math.toRadians(66.92),
							Math.toRadians(-61.34),
							Math.toRadians(-65.74),
							Math.toRadians(165.43));
				}
				else if (nTest_rotation==6) {
					jointPos_register =new JointPosition(   Math.toRadians(-53.56),
							Math.toRadians(39.95),
							Math.toRadians(20.17),
							Math.toRadians(106.44),
							Math.toRadians(13.83),
							Math.toRadians(-40.23),
							Math.toRadians(105.35));
				}
				else if (nTest_rotation==7) {
					jointPos_register =new JointPosition(   Math.toRadians(-36.18),
							Math.toRadians(39.19),
							Math.toRadians(20.16),
							Math.toRadians(108.88),
							Math.toRadians(-17.95),
							Math.toRadians(-41.29),
							Math.toRadians(145.14));
				}
				else if (nTest_rotation==8) {
					jointPos_register =new JointPosition(   Math.toRadians(-15.28),
							Math.toRadians(-13.82),
							Math.toRadians(20.16),
							Math.toRadians(45.57),
							Math.toRadians(-46.72),
							Math.toRadians(-94.51),
							Math.toRadians(137.60));
				}
				else if (nTest_rotation==9) {
					jointPos_register =new JointPosition(   Math.toRadians(-29.94),
							Math.toRadians(-22.34),
							Math.toRadians(20.17),
							Math.toRadians(45.91),
							Math.toRadians(-66.93),
							Math.toRadians(-76.02),
							Math.toRadians(154.30));
				}
				else if (nTest_rotation==10) {
					jointPos_register =new JointPosition(   Math.toRadians(-44.67),
							Math.toRadians(21.60),
							Math.toRadians(20.17),
							Math.toRadians(96.55),
							Math.toRadians(-34.25),
							Math.toRadians(-41.81),
							Math.toRadians(158.70));
				}
				else if (nTest_rotation==11) {
					jointPos_register =new JointPosition(   Math.toRadians(-34.67),
							Math.toRadians(4.73),
							Math.toRadians(20.16),
							Math.toRadians(75.22),
							Math.toRadians(-47.75),
							Math.toRadians(-57.78),
							Math.toRadians(174.88));
				}
				else if (nTest_rotation==12) {
					jointPos_register =new JointPosition(   Math.toRadians(-44.37),
							Math.toRadians(30.60),
							Math.toRadians(20.17),
							Math.toRadians(104.51),
							Math.toRadians(-0.52),
							Math.toRadians(-35.08),
							Math.toRadians(105.39));
				}
				else if (nTest_rotation==13) {
					jointPos_register =new JointPosition(   Math.toRadians(-28.23),
							Math.toRadians(13.50),
							Math.toRadians(20.17),
							Math.toRadians(83.06),
							Math.toRadians(-30.64),
							Math.toRadians(-53.64),
							Math.toRadians(157.66));
				}
				else if (nTest_rotation==14) {
					jointPos_register =new JointPosition(   Math.toRadians(-105.53),
							Math.toRadians(37.23),
							Math.toRadians(20.17),
							Math.toRadians(100.86),
							Math.toRadians(74.33),
							Math.toRadians(-60.81),
							Math.toRadians(49.51));
				}
				///
				
				
				

				 Err="10,";
					lbr.move(ptp(jointPos_register).setJointVelocityRel(0.2));
					Err="0,";
					nTest_rotation++;
					if (nTest_rotation==15){
						nTest_rotation=0;
					}
					nWorkingmode=0;
			}
			else if(nWorkingmode==10){
				jointPos_register =new JointPosition(   Math.toRadians(-105.53),
						Math.toRadians(37.23),
						Math.toRadians(20.17),
						Math.toRadians(100.86),
						Math.toRadians(74.33),
						Math.toRadians(-60.81),
						Math.toRadians(49.51));
			}
			
			else{
				
//				
				if(DangerMove==false && nLastWorkingmode==4){
					JointPosition jReady =lbr.getCurrentJointPosition();
//	                final JointPosition currentPos = lbr.getCurrentJointPosition();
	                
	     		   
//					 if(Math.toDegrees(jReady.get(JointEnum.J1)) < -160 || Math.toDegrees(jReady.get(JointEnum.J2)) < -25 || Math.toDegrees(jReady.get(JointEnum.J3)) < -65 || Math.toDegrees(jReady.get(JointEnum.J4)) < -10 || Math.toDegrees(jReady.get(JointEnum.J5)) < -160 ||  Math.toDegrees(jReady.get(JointEnum.J6)) < -110 || Math.toDegrees(jReady.get(JointEnum.J7)) < -165 || Math.toDegrees(jReady.get(JointEnum.J1)) > 160 || Math.toDegrees(jReady.get(JointEnum.J2)) > 10 || Math.toDegrees(jReady.get(JointEnum.J3)) > 55 || Math.toDegrees(jReady.get(JointEnum.J4)) > 110 || Math.toDegrees(jReady.get(JointEnum.J5)) > 160 ||  Math.toDegrees(jReady.get(JointEnum.J6)) > 110 || Math.toDegrees(jReady.get(JointEnum.J7)) > 165){
//	                	System.out.println("dangermove1");
//	                	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"));
//				    	needle.getFrame("/zuo_21002_zhiPolish").move(lin(Ptest1).setJointVelocityRel(0.2));
//				    	DangerMove=true;
//	                }
					 
		                if(Math.toDegrees(jReady.get(JointEnum.J1)) < -160 || Math.toDegrees(jReady.get(JointEnum.J2)) < -34 || Math.toDegrees(jReady.get(JointEnum.J3)) < -65 || Math.toDegrees(jReady.get(JointEnum.J4)) < -10 || Math.toDegrees(jReady.get(JointEnum.J5)) < -160 ||  Math.toDegrees(jReady.get(JointEnum.J6)) < -110 || Math.toDegrees(jReady.get(JointEnum.J7)) < -165 || Math.toDegrees(jReady.get(JointEnum.J1)) > 160 || Math.toDegrees(jReady.get(JointEnum.J2)) > 70 || Math.toDegrees(jReady.get(JointEnum.J3)) > 55 || Math.toDegrees(jReady.get(JointEnum.J4)) > 115 || Math.toDegrees(jReady.get(JointEnum.J5)) > 160 ||  Math.toDegrees(jReady.get(JointEnum.J6)) > 110 || Math.toDegrees(jReady.get(JointEnum.J7)) > 165){
		                	if(Math.toDegrees(jReady.get(JointEnum.J1)) < -160 || Math.toDegrees(jReady.get(JointEnum.J1)) > 160){
		                		System.out.println("J1:"+jReady.get(JointEnum.J1)+"   MAX:160;MIN:-160");
		                	}
		                	if(Math.toDegrees(jReady.get(JointEnum.J2)) < -34 || Math.toDegrees(jReady.get(JointEnum.J2)) > 70){
		                		System.out.println("J2:"+jReady.get(JointEnum.J2)+"   MAX:70;MIN:-34");
		                	}
		                	if(Math.toDegrees(jReady.get(JointEnum.J3)) < -65 || Math.toDegrees(jReady.get(JointEnum.J3)) > 55){
		                		System.out.println("J3:"+jReady.get(JointEnum.J3)+"   MAX:55;MIN:-65");
		                	}
		                	if(Math.toDegrees(jReady.get(JointEnum.J4)) < -10 || Math.toDegrees(jReady.get(JointEnum.J4)) > 115){
		                		System.out.println("J4:"+jReady.get(JointEnum.J4)+"   MAX:110;MIN:-10");
		                	}
		                	if(Math.toDegrees(jReady.get(JointEnum.J5)) < -160 || Math.toDegrees(jReady.get(JointEnum.J5)) > 160){
		                		System.out.println("J5:"+jReady.get(JointEnum.J5)+"   MAX:160;MIN:-160");
		                	}
		                	if(Math.toDegrees(jReady.get(JointEnum.J6)) < -110 || Math.toDegrees(jReady.get(JointEnum.J6)) > 110){
		                		System.out.println("J6:"+jReady.get(JointEnum.J6)+"   MAX:110;MIN:-110");
		                	}
		                	if(Math.toDegrees(jReady.get(JointEnum.J7)) < -165 || Math.toDegrees(jReady.get(JointEnum.J7)) > 165){
		                		System.out.println("J7:"+jReady.get(JointEnum.J7)+"   MAX:165;MIN:-165");
		                	}
		                	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/zuo_21002_zhiPolish"));
					    	needle.getFrame("/zuo_21002_zhiPolish").move(lin(Ptest1).setJointVelocityRel(0.2));
					    	DangerMove=true;
		                	System.out.println("dangermove1");
		                }
		                	
					 
				}
				//ThreadUtil.milliSleep(4);
			}

		
		

		
		}
		//return "end get";
	}

}


	
	public void MotionType() {
		// TODO è‡ªåŠ¨ç”Ÿæˆ�çš„æ–¹æ³•å­˜æ ¹
		
	}

	
	//@SuppressWarnings("null")
	@Override
	public void run()  {
		JointPosition actPos = lbr.getCurrentJointPosition();
		
//		BreakTest.initialize();
//		BreakTest.run();

		ExecutorService executor = Executors.newCachedThreadPool();
		Future<String> add = executor.submit(new sendRTdata());
		Future<String> say = executor.submit(new motion());
		Future<String> sdd2 = executor.submit(new reciveRTdata());
		Future<String> breaktest = executor.submit(new BreakTestProgram());
        //Monitor();

		try {
			System.out.println(add.get());
			System.out.println(say.get());
			System.out.println(sdd2.get());
			System.out.println(breaktest.get());
		} catch (InterruptedException e) {

			e.printStackTrace();
		} catch (ExecutionException e) {

			e.printStackTrace();
		} 

		
		
		

	}
	@Override
	public void dispose(){
		try {
			ThreadUtil.milliSleep(1000);
			if(serverSocket!=null){
				serverSocket.close();
				serverSocket=null;
				System.out.println("222");
			}
			if(serverSocketSend!=null ){
				serverSocketSend.close();
				serverSocketSend=null;
				System.out.println("22222");
			}
			if(writer!=null ){
				writer.close();
				writer=null;
				System.out.println("3333");
			}
			if(outputStream!=null ){
				outputStream.close();
				outputStream=null;
				System.out.println("133333");
			}
			if(socket!=null ){
				socket.close();
				socket=null;
				System.out.println("233333");
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
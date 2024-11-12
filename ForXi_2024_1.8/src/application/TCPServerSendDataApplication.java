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
	private BrakeTestMonitorSampleApp BreakTest;

	@Inject
	private motion2 OnlyPlane;


	@Inject
	private  LBR lbr;

	private Tool _toolAttachedToLBR;
	private Tool _toolAttachedToLBR_you;
	private Tool _toolAttachedToLBR_you2;
	private Tool _toolAttachedToLBR_you3;
	private Tool _toolAttachedToLBR_zuo;
	private Tool _toolAttachedToLBR_zuo2;
	private Tool _toolAttachedToLBR_zuo3;
	
	private Tool _toolAttachedToLBR_zuo_hand;
	private Tool _toolAttachedToLBR_you_hand;
	
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

	private static final double FREQENCY = 0.1;
	private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 30;

	/////////////////////////// 
	private LoadData _loadData;
	private LoadData _loadData_you;
	private LoadData _loadData_you2;
	private LoadData _loadData_you3;
	private LoadData _loadData_zuo;
	private LoadData _loadData_zuo2;
	private LoadData _loadData_zuo3;
	private LoadData _loadData_zuo_hand;
	private LoadData _loadData_you_hand;

	private  double[] TRANSLATION_OF_TOOL = { -150.7, 0, 227.9 };
	private  double[] TRANSLATION_OF_TOOL_you = { -107.28,-156.48,90.6,2.62,-0.01,3.13 };
	private  double[] TRANSLATION_OF_TOOL_you2 = { -105.48,-156.48,90.61,2.62,-0.01,3.13 };
	private  double[] TRANSLATION_OF_TOOL_you3 = { -106.28,-146.48,98.50,2.62,-0.01,3.13 }; 
	private  double[] TRANSLATION_OF_TOOL_zuo = { -188.87,-20.61,101.06,-1.5721,3.1415,0.0031 };
	private  double[] TRANSLATION_OF_TOOL_zuo2 = { -188.87,-18.81,101.07,-1.5907,3.1415,-0.006 };
	private  double[] TRANSLATION_OF_TOOL_zuo3 = { -178.87,-19.54,101.06,-1.5907,3.1415,-0.006 };
	private  double[] TRANSLATION_OF_TOOL_zuo_hand={ -8,0,170,0,0,0 };
	private  double[] TRANSLATION_OF_TOOL_you_hand={ -7.21,-12.48,184.45,0,0,0 };

	private static final double MASS = 0;
	private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { -45, 2.7, 120.4 };
	private static final String TOOL_FRAME = "toolFrame";
	private static final String TOOL_FRAME_you = "toolFrame_you";
	private static final String TOOL_FRAME_you2 = "toolFrame_you2";
	private static final String TOOL_FRAME_you3 = "toolFrame_you3";
	private static final String TOOL_FRAME_zuo = "toolFrame_zuo";
	private static final String TOOL_FRAME_zuo2 = "toolFrame_zuo2";
	private static final String TOOL_FRAME_zuo3 = "toolFrame_zuo3";
	private static final String TOOL_FRAME_zuo_hand = "toolFrame_zuo_hand";
	private static final String TOOL_FRAME_you_hand = "toolFrame_you_hand";
	protected String Tool_you_string;
	protected String Tool_you2_string;
	protected String Tool_you3_string;
	protected String Tool_zuo_string;
	protected String Tool_zuo2_string;
	protected String Tool_zuo3_string;
	protected String Tool_zuo_hand_string;
	protected String Tool_you_hand_string;
	//////////////////////////


	private static final double[] MAX_TRANSLATION_VELOCITY = { 150, 150, 150 };
	private ISmartServoLINRuntime _smartServoLINRuntime = null;
	private static final int NUM_RUNS = 600;
	private static final double AMPLITUDE = 70;
	private SafeDataIOGroup SafeDataIO;
	private JointPosition jointPos;
	private JointPosition jointPos_zuo;
	private JointPosition jointPos_you;
	private JointPosition jointPos_register;
	private JointPosition jointPos_zuo_transition;
	private JointPosition jointPos_you_transition;
	public boolean bForPlane;
	public boolean bSendFail;
	public boolean StartOnece;
	public boolean Distal;
	public boolean bChange=false;
	public boolean transition=false;
//	public boolean test=true;
	public boolean electrify;
	TimeUnit uint;
	Frame pre_Place;
	Frame Tcp;
	int  num_ForTest=0;
	int count=0;
	//	@Named("gripper")
	//	@Inject
	//	private Tool gripper;

	//	@Named("Tool_2")
	//	@Inject
	//	private Tool needle_Tool_2;

	@Named("Tool_2")

	@Inject
	com.kuka.roboticsAPI.geometricModel.Tool needle_Tool_2;

	@Named("Tool_3")
	@Inject
	com.kuka.roboticsAPI.geometricModel.Tool needle_Tool_3;

	@Named("gripper")
	@Inject
	com.kuka.roboticsAPI.geometricModel.Tool needle_gripper;

	//
	private ObjectFrame zuo_21001;
	//
	private ObjectFrame tcp;
	private ObjectFrame you_21001;
	private ObjectFrame handle;
	private BooleanIOCondition VaccumDetect;
	private Frame startpos;
	private Tool tool;
	public boolean bDangerous=false;

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
	public static String data19="0";
	public static String data20="0";

	public double nintegral=0;
	public double nderivative=0;
	public double nPrevious_error=0;
	public double nOutput=0;
	public double nP=0.5;
	public double nI=0.02;
	public double nD=0.01;
	public double nDistance=0;
	public double nStiff=0;
	public double nTest=0;
	public double n_Test=1;
	public double nTest_rotation=0;

	//å…¨å±€X,Y,Zå�˜é‡� è¾“å…¥å�˜é‡�
	public static double nX;
	public static double nY;
	public static double nZ;
	public static double nA;
	public static double nB;
	public static double nC;
	public static double A;
	public static double B;
	public static double C;
	public static double A_cps=0;
	public static double B_cps=0;
	public static double C_cps=0;
	public String Err="0,";
	public boolean bCorrectPlane=false;
	public boolean breaktest=false;
	//	public boolean Initialize=false;
	JointPosition currentPos_test;
	double nXcorect;
	/////位置控制测试
	private double ft;
	private double fr;
	private double Fx;
	private double Fy;
	private double Fz;
	private double Fa;
	private double Fb;
	private double Fc;
	private double Fm = 20;
	private double TScale;
	private double RScale;
	private double Pt;
	private double Pr;
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
	private double fb;
	private double fc;
	private double dfa;
	private double dfb;
	private double dfc;
	private double pa;
	private double pb;
	private double pc;
	private double dx_Angle;
	private double dy_Angle;
	private double dz_Angle;
	private double zFx=-1;
	private double zFy=0;
	private double zFz=1;
	private double zFa=1;
	private double zFb=0;
	private double zFc=0;
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
	
	boolean bstop = false;
	////////////
	//å…¨å±€å·¥ä½œæ¨¡å¼�å�˜é‡� è¾“å…¥å�˜é‡�
	public static int nWorkingmode=0;
	public static int nToolMode=0;
	public static int nReceive=0;
	@Inject
	private CopyOfTeachingByHand_2 JointImpedanceMode;
	@Override
	public void initialize() {

		nXcorect=0;
		Distal=false;
		bCorrectPlane=false;
		breaktest=false;
		StartOnece=true;
		bSendFail=true;
		VaccumDetect = new BooleanIOCondition(io.getInput("Input4"), true);
		count=0;
		Err="0,";
		num_ForTest=0;
		OnlyPlane.initialize();
		bForPlane=false;
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
		io.setOutput5(false);
		jointPos=new JointPosition(   Math.toRadians(40.2),
				Math.toRadians(-13.02),
				Math.toRadians(-2.20),
				Math.toRadians(78.93),
				Math.toRadians(47.49),
				Math.toRadians(48.25),
				Math.toRadians(-113.84));

		jointPos_zuo=new JointPosition(   Math.toRadians(24.12),
				Math.toRadians(-12.20),
				Math.toRadians(8.82),
				Math.toRadians(87.29),
				Math.toRadians(32.92),
				Math.toRadians(54.79),
				Math.toRadians(-128.32));

		jointPos_you=new JointPosition(   Math.toRadians(-31.46),
				Math.toRadians(-4.32),
				Math.toRadians(6.14),
				Math.toRadians(91.34),
				Math.toRadians(-34.69),
				Math.toRadians(45.71),
				Math.toRadians(67.78));
		jointPos_zuo_transition=new JointPosition(   Math.toRadians(29.86),
				Math.toRadians(-2.14),
				Math.toRadians(7.67),
				Math.toRadians(74.85),
				Math.toRadians(45.34),
				Math.toRadians(34.49),
				Math.toRadians(-63.27));
		jointPos_you_transition=new JointPosition(   Math.toRadians(-5.85),
                Math.toRadians(-16.96),
                Math.toRadians(-53.00),
                Math.toRadians(62.36),
                Math.toRadians(-47.16),
                Math.toRadians(50.81),
                Math.toRadians(0.33));




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

		///////////////////////	
		lbr = getContext().getDeviceFromType(LBR.class);
		
//		_loadData = new LoadData();
//		_loadData.setMass(MASS);
//		_loadData.setCenterOfMass(
//		                CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
//		                CENTER_OF_MASS_IN_MILLIMETER[2]);
		//        
		//        _toolAttachedToLBR = new Tool("Tool", _loadData);
		//
		//        XyzAbcTransformation trans = XyzAbcTransformation.ofRad(TRANSLATION_OF_TOOL[0], TRANSLATION_OF_TOOL[1], TRANSLATION_OF_TOOL[2],TRANSLATION_OF_TOOL[3],TRANSLATION_OF_TOOL[4],TRANSLATION_OF_TOOL[5]);
		//        ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME + "(TCP)", trans);
		//        _toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
		//        // Attach tool to the robot
		//        _toolAttachedToLBR.attachTo(lbr.getFlange());
		//        _toolAttachedToLBR.attachTo(tool.getFrame("/toolFrame"));



		needle_Tool_2.attachTo(lbr.getFlange());
		//lhy
		needle_Tool_3.attachTo(lbr.getFlange());
		needle_gripper.attachTo(lbr.getFlange());
		////////////////////////////////
	}


	public  class sendRTdata implements Callable<String> {

		public void GetData()
		{
			//			 System.out.println("start_test");
			//		        ThreadUtil.milliSleep(5000);
			//		        System.out.println("end_test");
			//		        io.setOutput2(true);
			//æš‚æ—¶æ— æ„�ä¹‰ï¼ˆé¢„ç•™é»˜è®¤ä¸º0ï¼‰
			data0 = "$0,";

			//æŠ¥è­¦ä»£ç �
			data1 = "0,";

			//å·¥ä½œæ¨¡å¼�

			data2 = Err;
			//				System.out.println(Err);
			//æ˜¯å�¦ä¸Šç”µ
			data3 = "2,";

			//å½“å‰�é€Ÿåº¦
			//CartesianVelocityLimitInfo infoObject = lbr.getCartesianVelocityLimitInfo();
			double a1 =getApplicationControl().getApplicationOverride();
			BigDecimal bigDecimal0 = new BigDecimal(a1);
			a1 = bigDecimal0.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
			data4 = String.valueOf(a1)+",";
			///

			///
			//æ— æ„�ä¹‰
			data5 = "3,";

			//				//å…³èŠ‚åº¦æ•°1

			JointPosition actPos = lbr.getCurrentJointPosition();
			//				a1 = Math.toDegrees(actPos.get(0));
			//				BigDecimal bigDecimal = new BigDecimal(a1);
			//				a1 = bigDecimal.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
			//				data6=String.valueOf(a1)+",";
			if(electrify==true){
				a1=1;
				data6=String.valueOf(a1)+",";
				//					ThreadUtil.milliSleep(2000);
				//					System.out.println("data13:"+data13);
			}
			else if(electrify==false){
				a1=0;
				data6=String.valueOf(a1)+",";
				//					ThreadUtil.milliSleep(2000);

				//					System.out.println("data14:"+data14);
			}
			//				//å…³èŠ‚åº¦æ•°2
//			a1 = Math.toDegrees(actPos.get(1));
			a1 = (int)(Math.random()*100000+1);
			BigDecimal bigDecimal1 = new BigDecimal(a1);
			a1 = bigDecimal1.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
			data7=String.valueOf(a1)+",";

			//				//å…³èŠ‚åº¦æ•°3
			a1 = Math.toDegrees(actPos.get(2));
			BigDecimal bigDecimal2 = new BigDecimal(a1);
			a1 = bigDecimal2.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
			data8=String.valueOf(a1)+",";

			//å…³èŠ‚åº¦æ•°4
			a1 = Math.toDegrees(actPos.get(3));
			BigDecimal bigDecimal3 = new BigDecimal(a1);
			a1 = bigDecimal3.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
			data9=String.valueOf(a1)+",";

			//å…³èŠ‚åº¦æ•°5
			a1 = Math.toDegrees(actPos.get(4));
			BigDecimal bigDecimal4 = new BigDecimal(a1);
			a1 = bigDecimal4.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
			data10=String.valueOf(a1)+",";

			//å…³èŠ‚åº¦æ•°6
			a1 = Math.toDegrees(actPos.get(5));
			BigDecimal bigDecimal5 = new BigDecimal(a1);
			a1 = bigDecimal5.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
			data11=String.valueOf(a1)+",";

			//å…³èŠ‚åº¦æ•°7
			a1 = Math.toDegrees(actPos.get(6));
			BigDecimal bigDecimal6 = new BigDecimal(a1);
			a1 = bigDecimal6.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
			data12=String.valueOf(a1)+",";




			//è½´å��æ ‡x
			Frame cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/you_21001"));



			//				if (nToolMode==2){
			//					 cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_anfang"));
			//				}
			//				else{
			//					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001"));
			//				}

			if (nToolMode==1){
				cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/you_21001"));
				cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/you_21001"));
				//						System.out.println("1:"+cmdPos);
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/you_21001"), cmdPos2);


			}
			else if(nToolMode==2)
			{
				cmdPos = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you.getDefaultMotionFrame());
				cmdPos = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you.getDefaultMotionFrame());
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you.getDefaultMotionFrame(), cmdPos2);

			}	
			else if(nToolMode==3)
			{			
				cmdPos = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you2.getDefaultMotionFrame());
				cmdPos = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you2.getDefaultMotionFrame());
				//					System.out.println("1:"+cmdPos);
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you2.getDefaultMotionFrame(), cmdPos2);


			}	
			else if(nToolMode==4)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/tcp_x_1_yz3"));
			}	
			else if(nToolMode==5)
			{
				cmdPos = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you3.getDefaultMotionFrame());
				cmdPos = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you3.getDefaultMotionFrame());
				//					System.out.println("1:"+cmdPos);
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you3.getDefaultMotionFrame(), cmdPos2);

			}	
			else if(nToolMode==6)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/you_21005"));
			}	
			else if(nToolMode==7)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_3.getFrame("/zuo_21001"));
				cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_3.getFrame("/zuo_21001"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle_Tool_3.getFrame("/zuo_21001"), cmdPos2);
			}	
			else if(nToolMode==8)
			{
				cmdPos = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo.getDefaultMotionFrame());
				cmdPos = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo.getDefaultMotionFrame());
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo.getDefaultMotionFrame(), cmdPos2);
			}	
			else if(nToolMode==9)
			{
				cmdPos = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo2.getDefaultMotionFrame());
				cmdPos = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo2.getDefaultMotionFrame());
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo2.getDefaultMotionFrame(), cmdPos2);
			}	
			else if(nToolMode==10)
			{
				cmdPos = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo3.getDefaultMotionFrame());
				cmdPos = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo3.getDefaultMotionFrame());
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCommandedCartesianPosition(_toolAttachedToLBR_zuo3.getDefaultMotionFrame(), cmdPos2);

				//					System.out.println("2:"+cmdPos);
			}	
			else if(nToolMode==11)
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_3.getFrame("/zuo_21005"));
				cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_3.getFrame("/zuo_21005"));
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCurrentCartesianPosition(needle_Tool_3.getFrame("/zuo_21005"), cmdPos2);
			}	
			else if(nToolMode==12)
			{
				cmdPos = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame());
				cmdPos = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame());
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCommandedCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame(), cmdPos2);

				//					System.out.println("2:"+cmdPos);
			}		
			else if(nToolMode==13)
			{
				cmdPos = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame());
				cmdPos = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame());
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCommandedCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame(), cmdPos2);

				//					System.out.println("2:"+cmdPos);
			}
			else
			{
				cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/test"));
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
							bSendFail=true;
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
							bSendFail=false;
							writer.close();
							outputStream.close();
							socket.close();
							serverSocketSend.close();
							writer=null;
							outputStream=null;
							socket=null;
							serverSocketSend=null;
							bPause=true;

							//					try {
							//						ThreadUtil.milliSleep(1000);
							//						if(serverSocket!=null){
							//							serverSocket.close();
							//							serverSocket=null;
							//							System.out.println("222");
							//						}
							//						if(serverSocketSend!=null ){
							//							serverSocketSend.close();
							//							serverSocketSend=null;
							//							System.out.println("22222");
							//						}
							//						if(writer!=null ){
							//							writer.close();
							//							writer=null;
							//							System.out.println("3333");
							//						}
							//						if(outputStream!=null ){
							//							outputStream.close();
							//							outputStream=null;
							//							System.out.println("133333");
							//						}
							//						if(socket!=null ){
							//							socket.close();
							//							socket=null;
							//							System.out.println("233333");
							//						}
							//					} catch (IOException f) {
							//						// TODO Auto-generated catch block
							//						e.printStackTrace();
							//					}

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

	public  class reciveRTdata implements Callable<String> {



		public String call() {	
			String[] units=null;
			while(true){
				try{
					boolean bPause=false;
					serverSocket = new ServerSocket(30007);

					System.out.println("New socket.");
					socket_recive = serverSocket.accept();
					socket_recive.setSoTimeout(5000);
					System.out.println("Socket accepted. IP:{" + socket_recive.getInetAddress().getHostAddress() + "}.");

					InputStream in = socket_recive.getInputStream();
					//			outputStream_recive
					outputStream_recive = new DataOutputStream(socket_recive.getOutputStream());
					writer_recive= new OutputStreamWriter(outputStream_recive);
					//			PrintWriter writer = new PrintWriter(new OutputStreamWriter(socket.getOutputStream()));

					while(bPause==false){

						try{

							//					byte[] receivingBuffer = new byte[256];
							//					in.read(receivingBuffer);

							byte[] buf = new byte[1];
							int size = 0;
							//				        System.out.println("qq");
							StringBuffer sb = new StringBuffer();

							//				        System.out.println(in.read(buf,0,buf.length));
							if (in.read(buf,0,buf.length)==-1)
							{
								bPause=true;
								System.out.println("oo");
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
									//								System.out.println("ooo");
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
									bstop=true;
									String para1 = units[7].substring(0, units[7].length() - 1);
									System.out.println("para" + para1);

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
									//System.out.println("pre_Place"+pre_Place);
									//System.out.println("nX"+nX+"  nY"+nY+"  nZ"+nZ+"  nA"+nA+"  nB"+nB+"  nC"+nC);
									writer_recive.write("$para,mp,0$");
									writer_recive.flush();
								}
								else if(units[1].equals("ml")){
									System.out.println("lhy-ml");
									String para1 = units[7].substring(0, units[7].length() - 1);
									System.out.println("zzr-ml");


									nX=Double.parseDouble(units[2]);
									nY=Double.parseDouble(units[3]);
									nZ=Double.parseDouble(units[4]);
									nA=Double.parseDouble(units[5]);
									nB=Double.parseDouble(units[6]);
									nC=Double.parseDouble(para1);
									A=Math.toDegrees(nA);
									B=Math.toDegrees(nB);
									C=Math.toDegrees(nC);
									//							System.out.println("A"+A);
									////							System.out.println("nX"+nX );
									////							System.out.println("nY"+nY );
									////							System.out.println("nZ"+nZ );
									//							System.out.println("nA"+nA );
									//							System.out.println("nB"+nB );
									//							System.out.println("nC"+nC );
									//							System.out.println("A"+A);
									//							System.out.println("B"+B);
									//							System.out.println("C"+C);
									//							pre_Place = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
									//							pre_Place.setX(nX);
									//							pre_Place.setY(nY);
									//							pre_Place.setZ(nZ);
									//							pre_Place.setAlphaRad(Math.toRadians(nA));
									//							pre_Place.setBetaRad(Math.toRadians(nB));
									//							pre_Place.setGammaRad(Math.toRadians(nC));

									writer_recive.write("$para,ml,0$");
									writer_recive.flush();
									ThreadUtil.milliSleep(1500);
									//							System.out.println("mlout");
								}

								else if(units[1].equals("mj")){
									//							System.out.println("chenpeng:11111111111 " );
									String para1 = units[7].substring(0, units[7].length() - 1);
									//							System.out.println("j1: " + units[2]);

									//确保handguing模式下是关掉的状态
									if(SafeDataIO.getInput4()==false){
										if(units[2].equals("1")){
											io.setOutput2(false);
											System.out.println("io.setOutput2(true)");
										}
										else if(units[2].equals("2")){
											io.setOutput2(true);
											System.out.println("io.setOutput2(false)");
										}
										if(units[3].equals("6")){
											nWorkingmode=6;
											System.out.println("nWorkingmode=6");
										}
									}


									//							nX=Double.parseDouble(units[2]);
									//							nY=Double.parseDouble(units[3]);
									//							nZ=Double.parseDouble(units[4]);
									//							nA=Double.parseDouble(units[5]);
									//							nB=Double.parseDouble(units[6]);
									//							nC=Double.parseDouble(para1);
									//							
									//							//ss
									//
									//							
									//							pre_Place = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
									//							pre_Place.setX(nX);
									//							pre_Place.setY(nY);
									//							pre_Place.setZ(nZ);
									//							pre_Place.setAlphaRad(Math.toRadians(nA));
									//							pre_Place.setBetaRad(Math.toRadians(nB));
									//							pre_Place.setGammaRad(Math.toRadians(nC));
									//							System.out.println("pre_Place"+pre_Place);
									//							System.out.println("nX"+nX+"  nY"+nY+"  nZ"+nZ+"  nA"+nA+"  nB"+nB+"  nC"+nC);
									writer_recive.write("$para,mp,0$");
									writer_recive.flush();
								}
								else if(units[1].equals("sIO")){
									//							System.out.println("chenpeng:tttt" );

									String para2 = units[3].substring(0, units[3].length() - 1);
									//							System.out.println("units[2]:" + Double.parseDouble(units[2]));
									//							System.out.println("para2:" + Double.parseDouble(para2));


									if(Double.parseDouble(para2)==12 && Double.parseDouble(units[2])==12)
									{
										bCorrectPlane=true;
										//							System.out.println("bCorrectPlane=true");

									}

									else if(Double.parseDouble(para2)==13 && Double.parseDouble(units[2])==13)
									{
										bCorrectPlane=false;
										//							System.out.println("bCorrectPlane=false");
									}
									else if(Double.parseDouble(para2)==14 && Double.parseDouble(units[2])==14)
									{
										//							Initialize=true;
										nXcorect=0;
										//							System.out.println("nXcorect=0");
									}

									else if(Double.parseDouble(para2)==19 && Double.parseDouble(units[2])==19)
									{
										Distal=true;
										//							System.out.println("Distal=true");
									}

									else if(Double.parseDouble(para2)==20 && Double.parseDouble(units[2])==20)
									{
										breaktest=true;
										//							System.out.println("breaktest=true");
									}
									else if(Double.parseDouble(para2)==21 && Double.parseDouble(units[2])==21)
									{
										transition=true;
//										test=false;
										System.out.println("transition=true");
									}
									else if(Double.parseDouble(para2)==22 && Double.parseDouble(units[2])==22)
									{
										System.out.println("Communication is normal");
										Err="13,";
									}
									//							if(Double.parseDouble(para2)==15 && Double.parseDouble(units[2])==15)
									//							{
									//							
									//							StartOnece=true;
									//							nXcorect=0;
									//							
									//							System.out.println("15");
									//							}

									if(Double.parseDouble(para2)==1 && Double.parseDouble(units[2])==1)
									{
										io.setOutput5(true);
										electrify=true;
//										System.out.println("on");
									}
									else{
										io.setOutput5(false);
										electrify=false;
//										System.out.println("off");

									}

									if(SafeDataIO.getInput4()==false){
										if(units[2].equals("2")){
											io.setOutput2(false);
											//		                            	System.out.println("io.setOutput2(false)");
										}
										else if(units[2].equals("3")){
											io.setOutput2(true);
											//		                            	System.out.println("io.setOutput2(true)");
										}

										if(para2.equals("6")){
											nWorkingmode=6;
										   System.out.println("nWorkingmode=6ch");
										}
									}


									writer_recive.write("$res,sIO,0$");
									writer_recive.flush();

								}
								else if(units[1].equals("RobotMove")){
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
										bstop=true;
									}
									else if(Double.parseDouble(para2)==4 ) {
										System.out.println("RobotMove4");
										nToolMode=4;
										bstop=true;
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
										bstop=true;
									}
									else if(Double.parseDouble(para2)==9 ) {
										System.out.println("RobotMove9");
										nToolMode=9;
										bstop=true;
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
									}
									//							else if(Double.parseDouble(para2)==13 ) {
									//								System.out.println("RobotMove13");
									//							}
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
							//					if (bSendFail==false){
							//						System.out.println("closed. bSendFail==false");					
							//						System.out.println("Socket. bSendFail==false");
							//						
							//						in.close();
							//						writer_recive.close();
							//						outputStream_recive.close();
							//						socket_recive.close();
							//						serverSocket.close();
							//						
							//						writer_recive=null;
							//						outputStream_recive=null;
							//						socket_recive=null;
							//						serverSocket=null;
							//						in=null;
							//						bPause=true;
							//				
							//					}

						}catch (IOException e ) {
							System.out.println("closed222.");					
							System.out.println("Socket222.");

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




						//				System.out.println("closed...");					
						//				System.out.println("Socket...");

					}
					System.out.println("closed111.");					
					System.out.println("Socket112.");

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



					//			in.close();
					//			writer_recive.close();
					//			outputStream_recive.close();
					//			socket_recive.close();
					//			serverSocket.close();
					//			
					//			writer_recive=null;
					//			outputStream_recive=null;
					//			socket_recive=null;
					//			serverSocket=null;
					//			in=null;
					bPause=true;

					System.out.println("closed000");					
					System.out.println("Socket000");
				}catch(IOException e){
					//				System.out.println("closed//...");		

					System.out.println("Socket//IOException e");
					ThreadUtil.milliSleep(1000);
					//				System.out.println("closed.");					
					//				System.out.println("Socket.");
					//				in.close();
					//				writer.close();
					//				socket.close();
					//				serverSocket.close();
					//				
					//				in=null;
					//				writer=null;
					//				//			outputStream=null;
					//				socket=null;
					//				serverSocket=null;
				};
				//			return "end";
			}
		}
	}


	public HandGuidingMotion createhandGuidingMotion(){

		double min1=0;
		double max1=0;
		JointPosition jReady =lbr.getCurrentJointPosition();
		if(Math.toDegrees(jReady.get(JointEnum.J1)) < -40){
			min1=Math.toDegrees(jReady.get(JointEnum.J1));
		}
		else
		{
			min1=-40;
		}

		if(Math.toDegrees(jReady.get(JointEnum.J1)) > 40){
			max1=Math.toDegrees(jReady.get(JointEnum.J1));
		}
		else
		{
			max1=40;
		}

		double min2=0;
		double max2=0;

		if(Math.toDegrees(jReady.get(JointEnum.J2)) < -60){
			min2=Math.toDegrees(jReady.get(JointEnum.J2));
		}
		else
		{
			min2=-60;
		}

		if(Math.toDegrees(jReady.get(JointEnum.J2)) > 60){
			max2=Math.toDegrees(jReady.get(JointEnum.J2));
		}
		else
		{
			max2=60;
		}


		double min3=0;
		double max3=0;

		if(Math.toDegrees(jReady.get(JointEnum.J3)) < -55){
			min3=Math.toDegrees(jReady.get(JointEnum.J3));
		}
		else
		{
			min3=-55;
		}

		if(Math.toDegrees(jReady.get(JointEnum.J3)) > 10){
			max3=Math.toDegrees(jReady.get(JointEnum.J3));
		}
		else
		{
			max3=10;
		}

		double min4=0;
		double max4=0;

		if(Math.toDegrees(jReady.get(JointEnum.J4)) < 50){
			min4=Math.toDegrees(jReady.get(JointEnum.J4));
		}
		else
		{
			min4=50;
		}

		if(Math.toDegrees(jReady.get(JointEnum.J4)) > 100){
			max4=Math.toDegrees(jReady.get(JointEnum.J4));
		}
		else
		{
			max4=100;
		}

		double min5=0;
		double max5=0;

		if(Math.toDegrees(jReady.get(JointEnum.J5)) < -150){
			min5=Math.toDegrees(jReady.get(JointEnum.J5));
		}
		else
		{
			min5=-150;
		}

		if(Math.toDegrees(jReady.get(JointEnum.J5)) > 50){
			max5=Math.toDegrees(jReady.get(JointEnum.J5));
		}
		else
		{
			max5=50;
		}

		double min6=0;
		double max6=0;

		if(Math.toDegrees(jReady.get(JointEnum.J6)) < 0){
			min6=Math.toDegrees(jReady.get(JointEnum.J6));
		}
		else
		{
			min6=0;
		}

		if(Math.toDegrees(jReady.get(JointEnum.J6)) > 100){
			max6=Math.toDegrees(jReady.get(JointEnum.J6));
		}
		else
		{
			max6=100;
		}

		double min7=0;
		double max7=0;

		if(Math.toDegrees(jReady.get(JointEnum.J7)) < -150){
			min7=Math.toDegrees(jReady.get(JointEnum.J7));
		}
		else
		{
			min7=-150;
		}

		if(Math.toDegrees(jReady.get(JointEnum.J7)) > 150){
			max7=Math.toDegrees(jReady.get(JointEnum.J7));
		}
		else
		{
			max7=150;
		}


		HandGuidingMotion motion = new HandGuidingMotion();
		motion.setJointVelocityLimit(0.8)
		.setCartVelocityLimit(900.0).setJointLimitViolationFreezesAll(false)
		.setJointLimitsMax(Math.toRadians(max1), Math.toRadians(max2), Math.toRadians(max3), Math.toRadians(max4), Math.toRadians(max5),Math.toRadians(max6), Math.toRadians(max7))
		.setJointLimitsMin(Math.toRadians(min1), Math.toRadians(min2), Math.toRadians(min3), Math.toRadians(min4), Math.toRadians(min5),Math.toRadians(min6), Math.toRadians(min7))
		.setJointLimitsEnabled(true,true,true,true,true,true,true);
		return motion;
	}


	private CartesianImpedanceControlMode createCartImp()
	    {
	    	final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
			cartImp.parametrize(CartDOF.X).setDamping(1);
			cartImp.parametrize(CartDOF.X).setStiffness(5000.0);
			cartImp.parametrize(CartDOF.Y).setStiffness(100.0);
			cartImp.parametrize(CartDOF.Z).setStiffness(200.0);
			cartImp.parametrize(CartDOF.ROT).setStiffness(250.0);
			cartImp.parametrize(CartDOF.C).setStiffness(30.0);
			cartImp.setNullSpaceStiffness(100.);
			cartImp.setMaxPathDeviation(1500., 1500., 1500., 3., 3., 3.);
	        return cartImp;
	    }



	public  class motion implements Callable<String> {


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
					//	            	System.out.println("xishu "+Math.toDegrees(Math.abs(Math.abs(Math.toRadians(170)) - Math.abs(currentPos_now.get(JointEnum.J5)))));

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
								//	                    	   System.out.println(para);
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

							//	                       System.out.println("xishu "+Math.toDegrees(Math.abs(Math.abs(Math.toRadians(170)) - Math.abs(currentPos_now.get(JointEnum.J5)))));
							jointImp.setStiffness(2000, 2000, 2000, 2000, para, para1, para2);
							//	                       System.out.println(para);
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


					//	                
					//	                if(Math.toDegrees(currentPos_now.get(JointEnum.J5))>160)
					//	                {
					//	                JointPosition currentPos1=new JointPosition(currentPos.get(JointEnum.J1), currentPos.get(JointEnum.J2), currentPos.get(JointEnum.J3), currentPos.get(JointEnum.J4), 2.7925, currentPos.get(JointEnum.J6), currentPos.get(JointEnum.J7));
					//	                destination.set(currentPos1);
					//	            	System.out.println("change.."+currentPos1);
					//	                }
					//	                else{
					//	                	destination.set(currentPos);
					//	                }

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
			//		        jointImp.setStiffness(1, 1, 1, 1, 1, 1, 1);

			jointImp.setDamping(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
			return jointImp;
		}

		protected CartesianImpedanceControlMode ConeLimit()
		{
			final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
			cartImp.parametrize(CartDOF.X).setStiffness(5000.0);
			cartImp.parametrize(CartDOF.Y).setStiffness(5000.0);
			cartImp.parametrize(CartDOF.Z).setStiffness(5000.0);
			cartImp.parametrize(CartDOF.ROT).setStiffness(300.0);

			//		        cartImp.parametrize(CartDOF.X).setAdditionalControlForce(-4.9);
			cartImp.setNullSpaceStiffness(100.);

			//		        cartImp.setMaxCartesianVelocity(5.0,5.0,5.0,0.2,0.2, 0.2);
			// For your own safety, shrink the motion abilities to useful limits
			//		        cartImp.setMaxControlForce(100.0, 100.0, 50.0, 20.0, 20.0, 20.0, true);
			//		        cartImp.setMaxControlForce(100.0, 100.0, 50.0, 20.0, 20.0, 20.0, true);
			//		        cartImp.setMaxControlForce(1, 1, 1, 1, 1, 1, true);
			cartImp.setMaxPathDeviation(150., 150., 150., 3., 3., 3.);
			return cartImp; 	
		}

		protected CartesianImpedanceControlMode HardLimit()
		{

			final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();

			if(nToolMode==3){
				cartImp.parametrize(CartDOF.X).setDamping(1);
				cartImp.parametrize(CartDOF.X).setStiffness(5000.0);
				cartImp.parametrize(CartDOF.Y).setStiffness(150.0);
				cartImp.parametrize(CartDOF.Z).setStiffness(200.0);
				cartImp.parametrize(CartDOF.ROT).setStiffness(250.0);
				cartImp.parametrize(CartDOF.C).setStiffness(30.0);
				cartImp.setNullSpaceStiffness(100.);
				cartImp.setMaxPathDeviation(1500., 1500., 1500., 3., 3., 3.);
			}
			else if(nToolMode==2){
				cartImp.parametrize(CartDOF.X).setDamping(1);
				cartImp.parametrize(CartDOF.X).setStiffness(5000.0);
				cartImp.parametrize(CartDOF.Y).setStiffness(150.0);
				cartImp.parametrize(CartDOF.Z).setStiffness(200.0);
				cartImp.parametrize(CartDOF.ROT).setStiffness(250.0);
				cartImp.parametrize(CartDOF.C).setStiffness(30.0);
				cartImp.setNullSpaceStiffness(100.);
				cartImp.setMaxPathDeviation(1500., 1500., 1500., 3., 3., 3.);
			}

			else if(nToolMode==8){
				cartImp.parametrize(CartDOF.X).setDamping(1);
				cartImp.parametrize(CartDOF.X).setStiffness(5000.0);
				cartImp.parametrize(CartDOF.Y).setStiffness(150.0);
				cartImp.parametrize(CartDOF.Z).setStiffness(200.0);
				cartImp.parametrize(CartDOF.ROT).setStiffness(250.0);
				cartImp.parametrize(CartDOF.C).setStiffness(30.0);
				
				
				cartImp.setNullSpaceStiffness(100.);
				cartImp.setMaxPathDeviation(1500., 1500., 1500., 3., 3., 3.);


			}
			else if(nToolMode==9){
				cartImp.parametrize(CartDOF.X).setDamping(1);
				cartImp.parametrize(CartDOF.X).setStiffness(5000.0);
				cartImp.parametrize(CartDOF.Y).setStiffness(150.0);
				cartImp.parametrize(CartDOF.Z).setStiffness(200.0);
				cartImp.parametrize(CartDOF.ROT).setStiffness(250.0);
				cartImp.parametrize(CartDOF.C).setStiffness(30.0);
				
				
				cartImp.setNullSpaceStiffness(100.);
				cartImp.setMaxPathDeviation(1500., 1500., 1500., 3., 3., 3.);

			}
			return cartImp; 	
		}
		protected CartesianImpedanceControlMode HardLimit_X()
		{

			final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();

			cartImp.parametrize(CartDOF.X).setStiffness(4500.0);
			cartImp.parametrize(CartDOF.Y).setStiffness(5000.0);
			cartImp.parametrize(CartDOF.Z).setStiffness(5000.0);
			cartImp.parametrize(CartDOF.ROT).setStiffness(300.0);

			//		        System.out.println(nStiff);
			//		        cartImp.parametrize(CartDOF.X).setAdditionalControlForce(-4.9);
			cartImp.setNullSpaceStiffness(100.);

			//		        cartImp.setMaxCartesianVelocity(5.0,5.0,5.0,0.2,0.2, 0.2);
			// For your own safety, shrink the motion abilities to useful limits
			//		        cartImp.setMaxControlForce(100.0, 100.0, 50.0, 20.0, 20.0, 20.0, true);
			//		        cartImp.setMaxControlForce(100.0, 100.0, 50.0, 20.0, 20.0, 20.0, true);
			//		        cartImp.setMaxControlForce(1, 1, 1, 1, 1, 1, true);
			cartImp.setMaxPathDeviation(1500., 1500., 1500., 3., 3., 3.);
			return cartImp; 	
		}
		protected CartesianImpedanceControlMode HardLimit_Y()
		{

			final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
			cartImp.parametrize(CartDOF.X).setDamping(1);
			cartImp.parametrize(CartDOF.X).setStiffness(5000.0);
			cartImp.parametrize(CartDOF.Y).setStiffness(150.0);
			cartImp.parametrize(CartDOF.Z).setStiffness(200.0);
			cartImp.parametrize(CartDOF.ROT).setStiffness(250.0);
			cartImp.parametrize(CartDOF.C).setStiffness(30.0);
			cartImp.setNullSpaceStiffness(100.);
			cartImp.setMaxPathDeviation(1500., 1500., 1500., 3., 3., 3.);

			return cartImp; 	
		}

		protected CartesianImpedanceControlMode HardLimit_Z()
		{

			final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();

			cartImp.parametrize(CartDOF.X).setStiffness(5000.0);
			cartImp.parametrize(CartDOF.Y).setStiffness(5000.0);
			cartImp.parametrize(CartDOF.Z).setStiffness(4500.0);
			cartImp.parametrize(CartDOF.ROT).setStiffness(300.0);

			//		        System.out.println(nStiff);
			//		        cartImp.parametrize(CartDOF.X).setAdditionalControlForce(-4.9);
			cartImp.setNullSpaceStiffness(100.);

			//		        cartImp.setMaxCartesianVelocity(5.0,5.0,5.0,0.2,0.2, 0.2);
			// For your own safety, shrink the motion abilities to useful limits
			//		        cartImp.setMaxControlForce(100.0, 100.0, 50.0, 20.0, 20.0, 20.0, true);
			//		        cartImp.setMaxControlForce(100.0, 100.0, 50.0, 20.0, 20.0, 20.0, true);
			//		        cartImp.setMaxControlForce(1, 1, 1, 1, 1, 1, true);
			cartImp.setMaxPathDeviation(1500., 1500., 1500., 3., 3., 3.);
			return cartImp; 	
		}


		//ç¬›å�¡å°”é˜»æŠ—æŽ§åˆ¶ä»£ç �
		public void runSmartCartesianMotion(final IMotionControlMode controlMode){
			//		     	_loadData = new LoadData();
			//		        _loadData.setMass(MASS);
			//		        _loadData.setCenterOfMass(
			//		                CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
			//		                CENTER_OF_MASS_IN_MILLIMETER[2]);
			//		        _toolAttachedToLBR = new Tool("Tool", _loadData);
			//
			//		        XyzAbcTransformation trans = XyzAbcTransformation.ofTranslation(
			//		                TRANSLATION_OF_TOOL[0], TRANSLATION_OF_TOOL[1],
			//		                TRANSLATION_OF_TOOL[2]);
			//		        ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME
			//		                + "(TCP)", trans);
			//		        _toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
			//		        // Attach tool to the robot
			//		        _toolAttachedToLBR.attachTo(lbr.getFlange());

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
				//		         	destination.set(currentPos_test);
				destination.set(currentPos);

				boolean bDanger=false;
				//		         	System.out.println("hhh");
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

					//		                // Timing - draw one step
					//		                final OneTimeStep aStep = timing.newTimeStep();
					//		                // ///////////////////////////////////////////////////////
					//		                // Insert your code here
					//		                // e.g Visual Servoing or the like
					//		                // emulate some computational effort - or waiting for external
					//		                // stuff
					//		                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
					//
					//		                theServoRuntime.updateWithRealtimeSystem();
					//
					//		                final double curTime = System.nanoTime() - startTimeStamp;
					//		                final double sinArgument = omega * curTime;
					//
					//		                for (int k = 0; k < destination.getAxisCount(); ++k)
					//		                {
					//		                    destination.set(k, Math.sin(sinArgument)
					//		                            * AMPLITUDE + initialPosition.get(k));
					//		                }
					//		                theServoRuntime
					//		                        .setDestination(destination);
					//
					//		                // Modify the stiffness settings every now and then               
					//		                if (i % (NUM_RUNS / 10) == 0)
					//		                {
					//		                    // update realtime system
					//		                    if (controlMode instanceof CartesianImpedanceControlMode)
					//		                    {
					//		                        final CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) controlMode;
					//		                        final double aTransStiffVal = Math.max(100. * (i
					//		                                / (double) NUM_RUNS + 1), 1000.);
					//		                        final double aRotStiffVal = Math.max(10. * (i
					//		                                / (double) NUM_RUNS + 1), 150.);
					//		                        cartImp.parametrize(CartDOF.TRANSL).setStiffness(aTransStiffVal);
					//		                        cartImp.parametrize(CartDOF.ROT).setStiffness(aRotStiffVal);
					//		                        // Send the new Stiffness settings down to the
					//		                        // controller
					//		                        theServoRuntime
					//		                                .changeControlModeSettings(cartImp);
					//		                    }
					//		                }
					//		                aStep.end();
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
				Frame Ptest2 = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/you_21001"));

				while(nWorkingmode==2)
				{

					//testdata x:735  y:7.59  z:122 Aï¼š-91 Bï¼š-40 Cï¼š-178 $cmd,ml,715,7,122,-91,-40,-178$
					//$cmd,RobotMove,1$
					if(nX!=0 && nY!=0 && nZ!=0 ){
						Ptest2.setX(nX);
						Ptest2.setY(nY);
						Ptest2.setZ(nZ);
						//								System.out.println("nx:"+nX+"  ny:"+nY+"  nz:"+nZ+"  a:"+nZ);
						Ptest2.setAlphaRad(Math.toRadians(nA));
						Ptest2.setBetaRad(Math.toRadians(nB));
						Ptest2.setGammaRad(Math.toRadians(nC));

						//								ThreadUtil.milliSleep(500);
						//								System.out.println("222");
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
			//			int answer;
			//			answer = getApplicationUI().displayModalDialog(
			//			ApplicationDialogType.INFORMATION,"Moving Mode", "Manule","Handle");
			boolean DangerMove=false;
			int nLastWorkingmode=0;
			Frame Ptest_ForPlane = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/tcp_x_1_yz3"));
			Frame Ptest_ForPlane1 = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/tcp_x_1_yz3"));
			while (true)
			{ 



				boolean btest=SafeDataIO.getInput4();
				if (btest==true)
				{
					nWorkingmode=1;


				}
				if (nWorkingmode==1){
					transition=false;
					Err="0,";
					System.out.println("start handguiding");
					ThreadUtil.milliSleep(500);
					nLastWorkingmode=nWorkingmode;
					if(SafeDataIO.getInput4()==true)
					{
						bForPlane=true;
						ThreadUtil.milliSleep(500);
						DangerMove=false;
						if (nToolMode==2)
						{
							//nToolMode==2指的是安放模式
							needle_gripper.getFrame("/tcp").move(createhandGuidingMotion());
							bDangerous=false;
							nWorkingmode=0;
						}
						else{
							//其他指的是打磨模式
							needle_Tool_2.getFrame("/you_21001").move(createhandGuidingMotion());
							bDangerous=false;
							nWorkingmode=0;
						}

						nX=0;
						nY=0;
						nZ=0;
						nA=0;
						nB=0;
						nC=0;
					}
					nWorkingmode=0;
					StartOnece=true;
				}
				else if (nWorkingmode==2 ){
					nLastWorkingmode=nWorkingmode;
					bstop=true;
					lbr.move(new PTP(jointPos_zuo).setJointVelocityRel(0.2));
					nWorkingmode=0;

				}

				else if(nWorkingmode==3){
					nLastWorkingmode=nWorkingmode;
					bstop=true;
					lbr.move(new PTP(jointPos_you).setJointVelocityRel(0.2));
					nWorkingmode=0;
				}

				else if(nWorkingmode==5){
					nLastWorkingmode=nWorkingmode;
					System.out.println("nWorkingmode==5");

					Frame Tcp = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Tcp.setX(nX);
					Tcp.setY(nY);
					Tcp.setZ(nZ);
					Tcp.setAlphaRad(Math.toRadians(nA));
					Tcp.setBetaRad(Math.toRadians(nB));
					Tcp.setGammaRad(Math.toRadians(nC));


					Tool_you_string = "Tool_you_" + nReceive;
					Tool_you2_string ="Tool_you2_" + nReceive;
					Tool_you3_string ="Tool_you3_" + nReceive;
					Tool_zuo_string ="Tool_zuo_" + nReceive;
					Tool_zuo2_string ="Tool_zuo2_" + nReceive;
					Tool_zuo3_string ="Tool_zuo3_" + nReceive;
					Tool_you_hand_string = "Tool_you_hand_" + nReceive;
					Tool_zuo_hand_string = "Tool_zuo_hand_" + nReceive;

					//nToolMode==2   you_21002 下表面				
					_loadData_you = new LoadData();
					_loadData_you.setMass(MASS);
					_loadData_you.setCenterOfMass(
							CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
							CENTER_OF_MASS_IN_MILLIMETER[2]);
					//			

					_toolAttachedToLBR_you = new Tool(Tool_you_string, _loadData_you);

					XyzAbcTransformation trans2 = XyzAbcTransformation.ofRad(TRANSLATION_OF_TOOL_you[0], TRANSLATION_OF_TOOL_you[1], TRANSLATION_OF_TOOL_you[2],TRANSLATION_OF_TOOL_you[3],TRANSLATION_OF_TOOL_you[4],TRANSLATION_OF_TOOL_you[5]);

					ObjectFrame aTransformation2 = _toolAttachedToLBR_you.addChildFrame(TOOL_FRAME_you + "(TCP)", trans2);
					_toolAttachedToLBR_you.setDefaultMotionFrame(aTransformation2);
					//				    // Attach tool to the robot
					_toolAttachedToLBR_you.attachTo(lbr.getFlange());
					//					 System.out.println("_loadData_you");




					//				  //nToolMode==3   you_21003 上表面
					_loadData_you2 = new LoadData();
					_loadData_you2.setMass(MASS);
					_loadData_you2.setCenterOfMass(
							CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
							CENTER_OF_MASS_IN_MILLIMETER[2]);

					_toolAttachedToLBR_you2 = new Tool(Tool_you2_string, _loadData_you2);

					XyzAbcTransformation trans3 = XyzAbcTransformation.ofRad(TRANSLATION_OF_TOOL_you2[0], TRANSLATION_OF_TOOL_you2[1], TRANSLATION_OF_TOOL_you2[2],TRANSLATION_OF_TOOL_you2[3],TRANSLATION_OF_TOOL_you2[4],TRANSLATION_OF_TOOL_you2[5]);
					ObjectFrame aTransformation3 = _toolAttachedToLBR_you2.addChildFrame(TOOL_FRAME_you2 + "(TCP)", trans3);
					_toolAttachedToLBR_you2.setDefaultMotionFrame(aTransformation3);
					// Attach tool to the robot
					_toolAttachedToLBR_you2.attachTo(lbr.getFlange());
					//				    System.out.println("_loadData_you2");
					////					
					////				    
					////				    
					////				    
					////				    
					//				  //nToolMode==5     you_21004 验证点
					_loadData_you3 = new LoadData();
					_loadData_you3.setMass(MASS);
					_loadData_you3.setCenterOfMass(
							CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
							CENTER_OF_MASS_IN_MILLIMETER[2]);

					_toolAttachedToLBR_you3 = new Tool(Tool_you3_string, _loadData_you3);

					XyzAbcTransformation trans4 = XyzAbcTransformation.ofRad(TRANSLATION_OF_TOOL_you3[0], TRANSLATION_OF_TOOL_you3[1], TRANSLATION_OF_TOOL_you3[2],TRANSLATION_OF_TOOL_you3[3],TRANSLATION_OF_TOOL_you3[4],TRANSLATION_OF_TOOL_you3[5]);
					System.out.println("XyzAbcTransformation trans4"+trans4);
					ObjectFrame aTransformation4 = _toolAttachedToLBR_you3.addChildFrame(TOOL_FRAME_you3 + "(TCP)", trans4);
					_toolAttachedToLBR_you3.setDefaultMotionFrame(aTransformation4);
					// Attach tool to the robot
					_toolAttachedToLBR_you3.attachTo(lbr.getFlange());
					//				    System.out.println("_loadData_you3");





					//nToolMode==8     zuo_21002 下表面
					_loadData_zuo = new LoadData();
					_loadData_zuo.setMass(MASS);
					_loadData_zuo.setCenterOfMass(
							CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
							CENTER_OF_MASS_IN_MILLIMETER[2]);

					_toolAttachedToLBR_zuo = new Tool(Tool_zuo_string, _loadData_zuo);

					XyzAbcTransformation trans5 = XyzAbcTransformation.ofRad(TRANSLATION_OF_TOOL_zuo[0], TRANSLATION_OF_TOOL_zuo[1], TRANSLATION_OF_TOOL_zuo[2],TRANSLATION_OF_TOOL_zuo[3],TRANSLATION_OF_TOOL_zuo[4],TRANSLATION_OF_TOOL_zuo[5]);
					ObjectFrame aTransformation5 = _toolAttachedToLBR_zuo.addChildFrame(TOOL_FRAME_zuo + "(TCP)", trans5);
					_toolAttachedToLBR_zuo.setDefaultMotionFrame(aTransformation5);
					// Attach tool to the robot
					_toolAttachedToLBR_zuo.attachTo(lbr.getFlange());
					//				    System.out.println("_loadData_zuo");
					//				    
					//				    
					//				    
					//				    
					//				    
					//nToolMode==9   zuo_21003 上表面
					_loadData_zuo2 = new LoadData();
					_loadData_zuo2.setMass(MASS);
					_loadData_zuo2.setCenterOfMass(
							CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
							CENTER_OF_MASS_IN_MILLIMETER[2]);

					_toolAttachedToLBR_zuo2 = new Tool(Tool_zuo2_string, _loadData_zuo2);

					XyzAbcTransformation trans6 = XyzAbcTransformation.ofRad(TRANSLATION_OF_TOOL_zuo2[0], TRANSLATION_OF_TOOL_zuo2[1], TRANSLATION_OF_TOOL_zuo2[2],TRANSLATION_OF_TOOL_zuo2[3],TRANSLATION_OF_TOOL_zuo2[4],TRANSLATION_OF_TOOL_zuo2[5]);
					ObjectFrame aTransformation6 = _toolAttachedToLBR_zuo2.addChildFrame(TOOL_FRAME_zuo2 + "(TCP)", trans6);
					_toolAttachedToLBR_zuo2.setDefaultMotionFrame(aTransformation6);
					// Attach tool to the robot
					_toolAttachedToLBR_zuo2.attachTo(lbr.getFlange());
					//				    System.out.println("_loadData_zuo2");
					//				    
					//				    
					//				    
					//				    
					//				    
					//nToolMode==10 zuo_21004
					_loadData_zuo3 = new LoadData();
					_loadData_zuo3.setMass(MASS);
					_loadData_zuo3.setCenterOfMass(
							CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
							CENTER_OF_MASS_IN_MILLIMETER[2]);

					_toolAttachedToLBR_zuo3 = new Tool(Tool_zuo3_string, _loadData_zuo3);

					XyzAbcTransformation trans7 = XyzAbcTransformation.ofRad(TRANSLATION_OF_TOOL_zuo3[0], TRANSLATION_OF_TOOL_zuo3[1], TRANSLATION_OF_TOOL_zuo3[2],TRANSLATION_OF_TOOL_zuo3[3],TRANSLATION_OF_TOOL_zuo3[4],TRANSLATION_OF_TOOL_zuo3[5]);
					System.out.println("XyzAbcTransformation trans7"+trans7);
					ObjectFrame aTransformation7 = _toolAttachedToLBR_zuo3.addChildFrame(TOOL_FRAME_zuo3 + "(TCP)", trans7);
					_toolAttachedToLBR_zuo3.setDefaultMotionFrame(aTransformation7);
					// Attach tool to the robot
					_toolAttachedToLBR_zuo3.attachTo(lbr.getFlange());
					//				    System.out.println("_loadData_zuo3");


					//nToolMode==12 zuo_hand
					_loadData_zuo_hand = new LoadData();
					_loadData_zuo_hand.setMass(MASS);
					_loadData_zuo_hand.setCenterOfMass(
							CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
							CENTER_OF_MASS_IN_MILLIMETER[2]);

					_toolAttachedToLBR_zuo_hand = new Tool(Tool_zuo_hand_string, _loadData_zuo_hand);

					XyzAbcTransformation trans8 = XyzAbcTransformation.ofRad(TRANSLATION_OF_TOOL_zuo_hand[0], TRANSLATION_OF_TOOL_zuo_hand[1], TRANSLATION_OF_TOOL_zuo_hand[2],TRANSLATION_OF_TOOL_zuo_hand[3],TRANSLATION_OF_TOOL_zuo_hand[4],TRANSLATION_OF_TOOL_zuo_hand[5]);
					System.out.println("XyzAbcTransformation trans8"+trans8);
					ObjectFrame aTransformation8 = _toolAttachedToLBR_zuo_hand.addChildFrame(TOOL_FRAME_zuo_hand + "(TCP)", trans8);
					_toolAttachedToLBR_zuo_hand.setDefaultMotionFrame(aTransformation8);
					// Attach tool to the robot
					_toolAttachedToLBR_zuo_hand.attachTo(lbr.getFlange());
					//				    System.out.println("_loadData_zuo3");

					
					
					//nToolMode==13 you_hand
					_loadData_you_hand = new LoadData();
					_loadData_you_hand.setMass(MASS);
					_loadData_you_hand.setCenterOfMass(
							CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
							CENTER_OF_MASS_IN_MILLIMETER[2]);

					_toolAttachedToLBR_you_hand = new Tool(Tool_you_hand_string, _loadData_you_hand);

					XyzAbcTransformation trans9 = XyzAbcTransformation.ofRad(TRANSLATION_OF_TOOL_you_hand[0], TRANSLATION_OF_TOOL_you_hand[1], TRANSLATION_OF_TOOL_you_hand[2],TRANSLATION_OF_TOOL_you_hand[3],TRANSLATION_OF_TOOL_you_hand[4],TRANSLATION_OF_TOOL_you_hand[5]);
					System.out.println("XyzAbcTransformation trans9"+trans9);
					ObjectFrame aTransformation9 = _toolAttachedToLBR_you_hand.addChildFrame(TOOL_FRAME_you_hand + "(TCP)", trans9);
					_toolAttachedToLBR_you_hand.setDefaultMotionFrame(aTransformation9);
					// Attach tool to the robot
					_toolAttachedToLBR_you_hand.attachTo(lbr.getFlange());
					//				    System.out.println("_loadData_zuo3");
					
					//				    ThreadUtil.milliSleep(1500);
					nReceive++;
					nWorkingmode=0;
					System.out.println("5nWorkingmode=0;");

				}

				else if(nWorkingmode==6){
					bstop=true;
					nLastWorkingmode=nWorkingmode;
					transition=false;
					if(true){   
						//						if(num_ForTest!=0)
						if(true)
						{

//							if (transition==true){
//								System.out.println("transition==true");
//								if(nToolMode==2||nToolMode==3){
//
//				    				lbr.move(ptp(jointPos_you_transition).setJointVelocityRel(0.1).breakWhen(VaccumDetect));
//
//				    			}
//				    			else if(nToolMode==8||nToolMode==9){
//				    				lbr.move(ptp(jointPos_zuo_transition).setJointVelocityRel(0.1).breakWhen(VaccumDetect));
//				    			}
//
//								
//								JointPosition PosNow=lbr.getCurrentJointPosition();
//								if (nToolMode==2||nToolMode==3){
//				    				if (Math.abs(Math.toDegrees(PosNow.get(4))-Math.toDegrees(jointPos_you_transition.get(4)))<0.5 && Math.abs(Math.toDegrees(PosNow.get(5))-Math.toDegrees(jointPos_you_transition.get(5)))<0.5  && Math.abs(Math.toDegrees(PosNow.get(6))-Math.toDegrees(jointPos_you_transition.get(6)))<0.5){
//					    				transition=false;
////					    				System.out.println("transition=false turn");
//					    			}
//				    			}
//								else if(nToolMode==8||nToolMode==9){
//				    				if (Math.abs(Math.toDegrees(PosNow.get(4))-Math.toDegrees(jointPos_zuo_transition.get(4)))<0.5 && Math.abs(Math.toDegrees(PosNow.get(5))-Math.toDegrees(jointPos_zuo_transition.get(5)))<0.5  && Math.abs(Math.toDegrees(PosNow.get(6))-Math.toDegrees(jointPos_zuo_transition.get(6)))<0.5){
//					    				transition=false;
////					    				System.out.println("transition=false turn");
//					    			}
//				    			}
//
//							}

							if (transition==false){
//								System.out.println("num_ForTest!=0");
								Frame pre_Place1 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
								pre_Place1.setX(nX);
								pre_Place1.setY(nY);
								pre_Place1.setZ(nZ);
								pre_Place1.setAlphaRad(Math.toRadians(nA));
								pre_Place1.setBetaRad(Math.toRadians(nB));
								pre_Place1.setGammaRad(Math.toRadians(nC));	
								getLogger().info("目标点pre_Place1:"+pre_Place1);
								
								

								
								
								if(nToolMode==3){
									pre_Place1 = pre_Place1.transform((Transformation.ofDeg(0,0,0, -A, -B, -C)));
									pre_Place1 = pre_Place1.transform((Transformation.ofDeg(-TRANSLATION_OF_TOOL_you2[0] ,-TRANSLATION_OF_TOOL_you2[1],-TRANSLATION_OF_TOOL_you2[2], 0, 0, 0)));
									getLogger().info("变换后pre_Place1:"+pre_Place1);
								}
								else if(nToolMode==2){
									pre_Place1 = pre_Place1.transform((Transformation.ofDeg(0,0,0, -A, -B, -C)));
									pre_Place1 = pre_Place1.transform((Transformation.ofDeg(-TRANSLATION_OF_TOOL_you[0] ,-TRANSLATION_OF_TOOL_you[1],-TRANSLATION_OF_TOOL_you[2], 0, 0, 0)));
									getLogger().info("变换后pre_Place1:"+pre_Place1);
								}
								else if(nToolMode==8){
									pre_Place1 = pre_Place1.transform((Transformation.ofDeg(0,0,0, -A, -B, -C)));
									pre_Place1 = pre_Place1.transform((Transformation.ofDeg(-TRANSLATION_OF_TOOL_zuo[0] ,-TRANSLATION_OF_TOOL_zuo[1],-TRANSLATION_OF_TOOL_zuo[2], 0, 0, 0)));
									getLogger().info("变换后pre_Place1:"+pre_Place1);
								}
                                else if(nToolMode==9){
									pre_Place1 = pre_Place1.transform((Transformation.ofDeg(0,0,0, -A, -B, -C)));
									pre_Place1 = pre_Place1.transform((Transformation.ofDeg(-TRANSLATION_OF_TOOL_zuo2[0] ,-TRANSLATION_OF_TOOL_zuo2[1],-TRANSLATION_OF_TOOL_zuo2[2], 0, 0, 0)));
									getLogger().info("变换后pre_Place1:"+pre_Place1);
								}

								
							
							
								Frame pre_Place2 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
								pre_Place2.setX(nX);
								pre_Place2.setY(nY);
								pre_Place2.setZ(nZ);
								pre_Place2.setAlphaRad(Math.toRadians(nA));
								pre_Place2.setBetaRad(Math.toRadians(nB));
								pre_Place2.setGammaRad(Math.toRadians(nC));
																							

								Frame Ptest1 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you2.getDefaultMotionFrame());							
								Frame Object4 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you2.getDefaultMotionFrame());
								
								if(nToolMode==3){
						
									Ptest1 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you2.getDefaultMotionFrame());								
									Object4 = Ptest1.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
									Object4 = Object4.transform((Transformation.ofDeg(0,0,0, -A, -B, -C)));
									Object4 = Object4.transform((Transformation.ofDeg(-TRANSLATION_OF_TOOL_you2[0] ,-TRANSLATION_OF_TOOL_you2[1],-TRANSLATION_OF_TOOL_you2[2], 0, 0, 0)));

									
								}
								else if(nToolMode==2){
									Ptest1 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you.getDefaultMotionFrame());									
									getLogger().info("Ptest1:"+Ptest1);
									Object4 = Ptest1.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
									Object4 = Object4.transform((Transformation.ofDeg(0,0,0, -A, -B, -C)));
									Object4 = Object4.transform((Transformation.ofDeg(-TRANSLATION_OF_TOOL_you[0] ,-TRANSLATION_OF_TOOL_you[1],-TRANSLATION_OF_TOOL_you[2], 0, 0, 0)));
									getLogger().info("object4:"+Object4);
								}
								else if(nToolMode==9){
									Ptest1 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo2.getDefaultMotionFrame());
									getLogger().info("Ptest1:"+Ptest1);
									Object4 = Ptest1.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
									Object4 = Object4.transform((Transformation.ofDeg(0,0,0, -A, -B, -C)));
									Object4 = Object4.transform((Transformation.ofDeg(-TRANSLATION_OF_TOOL_zuo2[0] ,-TRANSLATION_OF_TOOL_zuo2[1],-TRANSLATION_OF_TOOL_zuo2[2], 0, 0, 0)));
									getLogger().info("object4:"+Object4);
								}
								else if(nToolMode==8){
									Ptest1 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo.getDefaultMotionFrame());
									getLogger().info("Ptest1:"+Ptest1);
									Object4 = Ptest1.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
									Object4 = Object4.transform((Transformation.ofDeg(0,0,0, -A, -B, -C)));
									Object4 = Object4.transform((Transformation.ofDeg(-TRANSLATION_OF_TOOL_zuo[0] ,-TRANSLATION_OF_TOOL_zuo[1],-TRANSLATION_OF_TOOL_zuo[2], 0, 0, 0)));
									getLogger().info("object4:"+Object4);
								}




								try{	
									JointPosition RobotJoint = lbr.getInverseKinematicFromFrameAndRedundancy(pre_Place1);
							
									double Joint4 = RobotJoint.get(JointEnum.J4)*180/Math.PI;
									
									getLogger().info("JointOf4:"+Joint4);									
									
									lbr.getInverseKinematicFromFrameAndRedundancy(Object4);
																		
									
									Frame Object5 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you2.getDefaultMotionFrame());

									if(nToolMode==3){
										Object5 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you2.getDefaultMotionFrame());
									}
									else if(nToolMode==2){
										Object5 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you.getDefaultMotionFrame());
									}
									else if(nToolMode==9){
										Object5 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo2.getDefaultMotionFrame());
									}
									else if(nToolMode==8){
										Object5 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo.getDefaultMotionFrame());
									}

									
									Object5.setX(Object4.getX());
									Object5.setY(Object4.getY());
									Object5.setZ(Object4.getZ());
									Object5.setAlphaRad(Object4.getAlphaRad());
									Object5.setBetaRad(Object4.getBetaRad());
									Object5.setGammaRad(Object4.getGammaRad());
									
									JointPosition test=lbr.getInverseKinematicFromFrameAndRedundancy(Object5);				
									
									if(Joint4<115.0){
										
										if(nToolMode==3){

											_toolAttachedToLBR_you2.getDefaultMotionFrame().move(lin(pre_Place2).setJointVelocityRel(0.1).breakWhen(VaccumDetect)); 										   										
														    		    	
										}
										else if(nToolMode==2){

											_toolAttachedToLBR_you.getDefaultMotionFrame().move(lin(pre_Place2).setJointVelocityRel(0.1).breakWhen(VaccumDetect));							
											   										
										}
										else if(nToolMode==9){

											_toolAttachedToLBR_zuo2.getDefaultMotionFrame().move(lin(pre_Place2).setJointVelocityRel(0.1).breakWhen(VaccumDetect));

										}
										else if(nToolMode==8){

											_toolAttachedToLBR_zuo.getDefaultMotionFrame().move(lin(pre_Place2).setJointVelocityRel(0.1).breakWhen(VaccumDetect));

										}
										
										
									}
									else{

										throw new Exception("当前位置定位J4超出安全范围");
									}
									

    									//更新平面定位初始点

    									if(nToolMode==3){
    										Ptest_ForPlane = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/tcp_x_1_yz3"));
    										Ptest_ForPlane1 = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/tcp_x_1_yz3"));
    										//System.out.println("lhy1");
    									}
    									else if(nToolMode==2){
    										Ptest_ForPlane = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/tcp_x_1_yz3"));
    										Ptest_ForPlane1 = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/tcp_x_1_yz3"));
    										//System.out.println("lhy2");
    									}
    									else if(nToolMode==9){
    										Ptest_ForPlane = lbr.getCurrentCartesianPosition(needle_Tool_3.getFrame("/tcp_xyz"));
    										Ptest_ForPlane1 = lbr.getCurrentCartesianPosition(needle_Tool_3.getFrame("/tcp_xyz"));
//    										System.out.println("lhy3");
    									}
    									else if(nToolMode==8){
    										Ptest_ForPlane = lbr.getCurrentCartesianPosition(needle_Tool_3.getFrame("/tcp_xyz"));
    										Ptest_ForPlane1 = lbr.getCurrentCartesianPosition(needle_Tool_3.getFrame("/tcp_xyz"));
//    										System.out.println("lhy4");
    									}

    									if(io.getInput4()==false){
    										Err="3,";
//    										System.out.println("Move");
    										Err="3,";
    										ThreadUtil.milliSleep(70);
    										Err="0,";
    									}
                                    

								}	
								catch(Throwable cause)
								{
									System.out.println("RangeLimit");
									Err="1,";
								}
							}
							else{

							}
						}
						else{
							num_ForTest=1;
							System.out.println("num_ForTest==0");
						}




						//	                    ThreadUtil.milliSleep(500);
						StartOnece=true;
						nWorkingmode=0;
						bstop=true;
					}



				}
				


				else if(nWorkingmode==7){
					System.out.println("start nWorkingmode=7");
					nLastWorkingmode=nWorkingmode;
					 final CartesianImpedanceControlMode carthard = HardLimit();
						
	                  if(nToolMode==2 || nToolMode==3){
				    		
					    	Frame cmdPos2 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame());
					    	
					    	JointPosition test =lbr.getCurrentJointPosition();	
					    	
			       	        Transformation DistanceToPlane=Ptest_ForPlane.staticTransformationTo(cmdPos2); 
			       	     
			       	        if(Math.abs(DistanceToPlane.getY())>160){
//						    	
						    		System.out.println("DistanceToPlane.getY())>140");
						    		cmdPos2 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame());
//					
						    		Ptest_ForPlane1.setX(cmdPos2.getX());
						    		Ptest_ForPlane1.setY(cmdPos2.getY());
						    		Ptest_ForPlane1.setZ(cmdPos2.getZ());
						    		Ptest_ForPlane1.setGammaRad(cmdPos2.getGammaRad());
						    		Ptest_ForPlane1.setBetaRad(cmdPos2.getBetaRad());
						    		Ptest_ForPlane1.setAlphaRad(cmdPos2.getAlphaRad());
						    		_toolAttachedToLBR_you_hand.getDefaultMotionFrame().move(ptp(cmdPos2).setJointVelocityRel(0.1));
									Ptest_ForPlane = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame());
									Ptest_ForPlane1 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame());
						
									Err="5,";
						    		nWorkingmode=0;
						    }
			       	     else if(Math.abs(DistanceToPlane.getZ())>150){
							    cmdPos2 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame());

							    		System.out.println("DgetZ()");
							    		Ptest_ForPlane1.setX(cmdPos2.getX());
							    		Ptest_ForPlane1.setY(cmdPos2.getY());
							    		Ptest_ForPlane1.setZ(cmdPos2.getZ());
							    		Ptest_ForPlane1.setGammaRad(cmdPos2.getGammaRad());
							    		Ptest_ForPlane1.setBetaRad(cmdPos2.getBetaRad());
							    		Ptest_ForPlane1.setAlphaRad(cmdPos2.getAlphaRad());
							 
							       
							    	_toolAttachedToLBR_you_hand.getDefaultMotionFrame().move(ptp(cmdPos2).setJointVelocityRel(0.1));
									Ptest_ForPlane = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame());
									Ptest_ForPlane1 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame());
						
									Err="5,";
							    	nWorkingmode=0;
							    }
							    else if(Math.toDegrees(test.get(JointEnum.J4)) > 115){
							        cmdPos2 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame());
//							   
							    		System.out.println("J4>115");
							    		Ptest_ForPlane1.setX(cmdPos2.getX());
							    		Ptest_ForPlane1.setY(cmdPos2.getY());
							    		Ptest_ForPlane1.setZ(cmdPos2.getZ());
							    		Ptest_ForPlane1.setGammaRad(cmdPos2.getGammaRad());
							    		Ptest_ForPlane1.setBetaRad(cmdPos2.getBetaRad());
							    		Ptest_ForPlane1.setAlphaRad(cmdPos2.getAlphaRad());
							

							    	_toolAttachedToLBR_you_hand.getDefaultMotionFrame().move(ptp(cmdPos2).setJointVelocityRel(0.1));

									Ptest_ForPlane = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame());
									Ptest_ForPlane1 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame());
									Err="5,";
									nWorkingmode=0;
									
							    }
							    else {
							    	
							    	_toolAttachedToLBR_you_hand.getDefaultMotionFrame().move(ptp(cmdPos2).setJointVelocityRel(0.2).setMode(carthard));
					       	       //_toolAttachedToLBR_you_hand.getDefaultMotionFrame().move(positionHold(carthard,0,uint));
					       	        nWorkingmode=0;
					       	        
							    }
			       	        
		    			    nWorkingmode=0;
				    	}
	                  else if(nToolMode==8 || nToolMode==9){
	                	   
					    	Frame cmdPos2 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame());
					    	JointPosition test =lbr.getCurrentJointPosition();		
					    	//System.out.println("JointEnum.J4:"+test.get(JointEnum.J4)*180/Math.PI); 
			       	        Transformation DistanceToPlane=Ptest_ForPlane.staticTransformationTo(cmdPos2); 
			       	     
			       	        if(Math.abs(DistanceToPlane.getY())>140){
//						    	
						    		System.out.println("DistanceToPlane.getY())>160");
						    		cmdPos2 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame());
//					
						    		Ptest_ForPlane1.setX(cmdPos2.getX());
						    		Ptest_ForPlane1.setY(cmdPos2.getY());
						    		Ptest_ForPlane1.setZ(cmdPos2.getZ());
						    		Ptest_ForPlane1.setGammaRad(cmdPos2.getGammaRad());
						    		Ptest_ForPlane1.setBetaRad(cmdPos2.getBetaRad());
						    		Ptest_ForPlane1.setAlphaRad(cmdPos2.getAlphaRad());
						    		_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame().move(ptp(cmdPos2).setJointVelocityRel(0.1));
									Ptest_ForPlane = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame());
									Ptest_ForPlane1 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame());
						
									Err="5,";
						    		nWorkingmode=0;
						    }
			       	     else if(Math.abs(DistanceToPlane.getZ())>150){
							    cmdPos2 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame());

							    		System.out.println("DgetZ()");
							    		Ptest_ForPlane1.setX(cmdPos2.getX());
							    		Ptest_ForPlane1.setY(cmdPos2.getY());
							    		Ptest_ForPlane1.setZ(cmdPos2.getZ());
							    		Ptest_ForPlane1.setGammaRad(cmdPos2.getGammaRad());
							    		Ptest_ForPlane1.setBetaRad(cmdPos2.getBetaRad());
							    		Ptest_ForPlane1.setAlphaRad(cmdPos2.getAlphaRad());
							 
							       
							    	_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame().move(ptp(cmdPos2).setJointVelocityRel(0.1));
									Ptest_ForPlane = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame());
									Ptest_ForPlane1 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame());
						
									Err="5,";
							    	nWorkingmode=0;
							    }
							    else if(Math.toDegrees(test.get(JointEnum.J4)) > 115){
							        cmdPos2 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame());
//							   
							    		System.out.println("J4>115");
							    		Ptest_ForPlane1.setX(cmdPos2.getX());
							    		Ptest_ForPlane1.setY(cmdPos2.getY());
							    		Ptest_ForPlane1.setZ(cmdPos2.getZ());
							    		Ptest_ForPlane1.setGammaRad(cmdPos2.getGammaRad());
							    		Ptest_ForPlane1.setBetaRad(cmdPos2.getBetaRad());
							    		Ptest_ForPlane1.setAlphaRad(cmdPos2.getAlphaRad());
							

							    	_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame().move(ptp(cmdPos2).setJointVelocityRel(0.1));

									Ptest_ForPlane = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame());
									Ptest_ForPlane1 = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame());
									Err="5,";
									nWorkingmode=0;
									
							    }
							    else {
							    	
							    	_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame().move(ptp(cmdPos2).setJointVelocityRel(0.2).setMode(carthard));
					       	        //_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame().move(positionHold(carthard,0,uint));
					       	        nWorkingmode=0;
					       	        
							    }
			       	        
		    			    nWorkingmode=0;
				    	}
	                  nWorkingmode=0;
				}
				else if(nWorkingmode==8){
					
					getLogger().info("workmode7");
		    	
					
                    if(nToolMode==2 || nToolMode==3)	{
                    	 AbstractFrame current = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame());
                     	
                     	
      				   _toolAttachedToLBR_you_hand.move(ptp(current).setJointVelocityRel(0.1));
      					    
      				   SmartServoLIN aSmartServoLINMotion = new SmartServoLIN(current);
      						
      					aSmartServoLINMotion.setMinimumTrajectoryExecutionTime(20e-3);
      					getLogger().info("2");
      						
      					_toolAttachedToLBR_you_hand.moveAsync(aSmartServoLINMotion.setMode(createCartImp()));
      					
      					getLogger().info("you");		
      						
      					ISmartServoLINRuntime smartServoLINRuntime = aSmartServoLINMotion.getRuntime();
      						
      						
      						
      						px = py = pz = 0.03;
      						
      						pa = pb = pc=0.0003;

      						Frame destFrame = current.copyWithRedundancy();
      						
      					
      						ccx = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getX();
      						ccy = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getY();
      						ccz = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getZ();
      						cca = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getAlphaRad();
      						ccb = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getBetaRad();
      						ccc = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getGammaRad();

      						JointPosition jStart=lbr.getCurrentJointPosition();
      						try {
      							while (!bstop) {
                                     
      								fx = lbr.getExternalForceTorque(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getForce()
      										.getX();
      								fy = lbr.getExternalForceTorque(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getForce()
      										.getY();
      								fz = lbr.getExternalForceTorque(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getForce()
      										.getZ();
      								fa = lbr.getExternalForceTorque(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getTorque().getZ();
      								fb = lbr.getExternalForceTorque(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getTorque().getY();
      								fc = lbr.getExternalForceTorque(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getTorque().getX();
      								
      								
      						  if (Math.abs(fy)>19 || Math.abs(fz)>19 || Math.abs(fc)>8){
      							  
      							
      								dfy = py * fy ;
      								dfz = -pz * fz; 
      								dfc =  pc * fc;
      								
      								cx = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getX();
      								cy = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getY();
      								cz = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getZ();
      								ca = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getAlphaRad();
      								cb = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getBetaRad();
      								cc = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getGammaRad();


      								JointPosition jNow=lbr.getCurrentJointPosition();
      								double bBiaozhi=1;
      								if (jNow.get(6)>0){
      									bBiaozhi=1;
      								}
      								else{
      									bBiaozhi=-1;
      								}
//      								
      									if(Math.abs(ccc-cc)>(11.5/57.2) && (ccc-cc)*fc<0 || Math.abs(dfc) < 0.001){
      										dfc = 0;
      									}
      									if(Math.toDegrees(jStart.get(6))<-170){
      										dfc = 0;	
      									}
      									if(Math.abs(ccx-cx)>80 && (ccx-cx)*fx>0 || Math.abs(dfx) < 0.001){
//      										
      										dfx = 0;
      									}
      									if(Math.abs(ccy-cy)>100 && (ccy-cy)*fy<0 || Math.abs(dfy) < 0.001){
//      								
      										dfy = 0;
      									}
      									if(Math.abs(ccz-cz)>80 && (ccz-cz)*fz>0 || Math.abs(dfz) < 0.001){
//      										
      										dfz = 0;
      									}
      									
//      								}


      								if (Math.abs(dfz) < 0.5) {
      									dfz = 0;
      								}
      								if (ccz - cz > 50) {
      									if (fz > 0) {
      										dfz = 0;

      									}
      								}

      								if (ccz - cz < -50) {
      									if (fz < 0) {
      										dfz = 0;

      									}

      								}


      									
      									Frame Ptest2 = destFrame.transform((Transformation.ofRad(0, dfy, -dfz, 0, 0, dfc)));
      									destFrame.setX(Ptest2.getX());
      									destFrame.setY(Ptest2.getY());
      									destFrame.setZ(Ptest2.getZ());
      									destFrame.setAlphaRad(Ptest2.getAlphaRad());
      									destFrame.setBetaRad(Ptest2.getBetaRad());
      									destFrame.setGammaRad(Ptest2.getGammaRad());

      								smartServoLINRuntime.updateWithRealtimeSystem();
      								smartServoLINRuntime.setDestination(destFrame);
      							
      							   }
      							}

      						}
      						 catch (Exception e) {
      								// TODO: handle exception
      								System.out.println(e.toString());
      							}
      						smartServoLINRuntime.stopMotion();  
                    }
                    else if (nToolMode==8 || nToolMode==9){
                         getLogger().info("3");
						
						AbstractFrame current = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame());
						
					    _toolAttachedToLBR_zuo_hand.move(ptp(current).setJointVelocityRel(0.1));
					    
					    SmartServoLIN aSmartServoLINMotion = new SmartServoLIN(current);
					    
						aSmartServoLINMotion.setMinimumTrajectoryExecutionTime(20e-3);
						
						_toolAttachedToLBR_zuo_hand.moveAsync(aSmartServoLINMotion.setMode(createCartImp()));
						
						getLogger().info("zuo");
						
						ISmartServoLINRuntime smartServoLINRuntime = aSmartServoLINMotion
								.getRuntime();
						
						
						
	                    pa = pb = pc=0.0003;
						

						Frame destFrame = current.copyWithRedundancy();
						
						ccx = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getX();
						ccy = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getY();
						ccz = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getZ();
						cca = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getAlphaRad();
						ccb = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getBetaRad();
						ccc = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getGammaRad();

						JointPosition jStart=lbr.getCurrentJointPosition();
						try {
							while (!bstop) {
								
								fx = lbr.getExternalForceTorque(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getForce()
										.getX();
								fy = lbr.getExternalForceTorque(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getForce()
										.getY();
								fz = lbr.getExternalForceTorque(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getForce()
										.getZ();
								fa = lbr.getExternalForceTorque(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getTorque().getZ();
								fb = lbr.getExternalForceTorque(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getTorque().getY();
								fc = lbr.getExternalForceTorque(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getTorque().getX();

					            cx = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getX();
								cy = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getY();
								cz = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getZ();
								ca = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getAlphaRad();
								cb = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getBetaRad();
								cc = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getGammaRad();

								
							 if (Math.abs(fy)>19 || Math.abs(fz)>19 || Math.abs(fc)>8){

								dfy = py * fy ;
								dfz = -pz * fz; 
								dfc =  pc * fc;


									JointPosition jNow=lbr.getCurrentJointPosition();
									double bBiaozhi=1;
									if (jNow.get(6)>0){
										bBiaozhi=1;
									}
									else{
										bBiaozhi=-1;
									}

										if(Math.abs(dfc) < 0.001){
											dfc = 0;					
										}
										
										if(Math.abs(Math.toDegrees(jStart.get(4))-Math.toDegrees(jNow.get(4)))>12 && (Math.toDegrees(jStart.get(4))-Math.toDegrees(jNow.get(4)))*fc*bBiaozhi<0  ){
											dfc = 0;	
										}
										
									
										if(Math.abs(ccy-cy)>100&& (ccy-cy)*fy<0 || Math.abs(dfy) < 0.001){
											dfy = 0;
										}
					
										if(Math.abs(ccz-cz)>75 && (ccz-cz)*fz>0 || Math.abs(dfz) < 0.001){
											dfz = 0;
										}



									if (Math.abs(dfz) < 0.5) {
										dfz = 0;
									}
									if (ccz - cz > 50) {
										if (fz > 0) {
											dfz = 0;

										}
									}

									if (ccz - cz < -50) {
										if (fz < 0) {
											dfz = 0;

										}

									}	
									
								Frame Ptest2 = destFrame.transform((Transformation.ofRad(0, dfy, -dfz, 0, 0, dfc)));
								destFrame.setX(Ptest2.getX());
								destFrame.setY(Ptest2.getY());
								destFrame.setZ(Ptest2.getZ());
								destFrame.setAlphaRad(Ptest2.getAlphaRad());
								destFrame.setBetaRad(Ptest2.getBetaRad());
								destFrame.setGammaRad(Ptest2.getGammaRad());
								
								smartServoLINRuntime.updateWithRealtimeSystem();
								smartServoLINRuntime.setDestination(destFrame);


//								// 获取最新的外部力信息
//								double[] externalForce =  smartServoLINRuntime.getExtForceMsr();


							 }
								
								
							}

						}
						 catch (Exception e) {
								
								System.out.println(e.toString());
							}
						smartServoLINRuntime.stopMotion();
                    }


				ThreadUtil.milliSleep(20);
					
					nWorkingmode=0;
					
				}
				else if(nWorkingmode==9){

					nLastWorkingmode=nWorkingmode;
					System.out.println("start nWorkingmode=9");
					if(nToolMode==2||nToolMode==3){
//						System.out.println("you");
						Frame current = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame());
						_toolAttachedToLBR_you_hand.move(ptp(current).setJointVelocityRel(0.1));


						SmartServoLIN aSmartServoLINMotion = new SmartServoLIN(current);
						aSmartServoLINMotion.setMinimumTrajectoryExecutionTime(20e-3);

//						System.out.println("Start Smart Servo Lin motion");

						//设置是否有阻抗的地方
						//_toolAttachedToLBR.moveAsync(aSmartServoLINMotion.setMode(cic));
						_toolAttachedToLBR_you_hand.moveAsync(aSmartServoLINMotion);

//						System.out.println("Get the runtime of the SmartServoLIN motion");
						ISmartServoLINRuntime smartServoLINRuntime = aSmartServoLINMotion
								.getRuntime();
                        //比例增益
						px = py = pz = 0.03;
					
						pa = pb = pc=0.0003;

						Frame destFrame = current.copyWithRedundancy();
						
					
						ccx = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getX();
						ccy = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getY();
						ccz = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getZ();
						cca = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getAlphaRad();
						ccb = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getBetaRad();
						ccc = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getGammaRad();

						JointPosition jStart=lbr.getCurrentJointPosition();
						try {
							while (!bstop) {
                               
								fx = lbr.getExternalForceTorque(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getForce()
										.getX();
								fy = lbr.getExternalForceTorque(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getForce()
										.getY();
								fz = lbr.getExternalForceTorque(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getForce()
										.getZ();
								fa = lbr.getExternalForceTorque(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getTorque().getZ();
								fb = lbr.getExternalForceTorque(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getTorque().getY();
								fc = lbr.getExternalForceTorque(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getTorque().getX();
								
								
						  if (Math.abs(fy)>19 || Math.abs(fz)>19 || Math.abs(fc)>8){
							  
							
								dfy = py * fy ;
								dfz = -pz * fz; 
								dfc =  pc * fc;
								
								cx = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getX();
								cy = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getY();
								cz = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getZ();
								ca = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getAlphaRad();
								cb = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getBetaRad();
								cc = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_you_hand.getDefaultMotionFrame()).getGammaRad();


								JointPosition jNow=lbr.getCurrentJointPosition();
								double bBiaozhi=1;
								if (jNow.get(6)>0){
									bBiaozhi=1;
								}
								else{
									bBiaozhi=-1;
								}
//								
									if(Math.abs(ccc-cc)>(11.5/57.2) && (ccc-cc)*fc<0 || Math.abs(dfc) < 0.001){
										dfc = 0;
									}
									if(Math.toDegrees(jStart.get(6))<-170){
										dfc = 0;	
									}
									if(Math.abs(ccx-cx)>80 && (ccx-cx)*fx>0 || Math.abs(dfx) < 0.001){
//										
										dfx = 0;
									}
									if(Math.abs(ccy-cy)>100 && (ccy-cy)*fy<0 || Math.abs(dfy) < 0.001){
//								
										dfy = 0;
									}
									if(Math.abs(ccz-cz)>80 && (ccz-cz)*fz>0 || Math.abs(dfz) < 0.001){
//										
										dfz = 0;
									}
									
//								}


								if (Math.abs(dfz) < 0.5) {
									dfz = 0;
								}
								if (ccz - cz > 50) {
									if (fz > 0) {
										dfz = 0;

									}
								}

								if (ccz - cz < -50) {
									if (fz < 0) {
										dfz = 0;

									}

								}


									
									Frame Ptest2 = destFrame.transform((Transformation.ofRad(0, dfy, -dfz, 0, 0, dfc)));
									destFrame.setX(Ptest2.getX());
									destFrame.setY(Ptest2.getY());
									destFrame.setZ(Ptest2.getZ());
									destFrame.setAlphaRad(Ptest2.getAlphaRad());
									destFrame.setBetaRad(Ptest2.getBetaRad());
									destFrame.setGammaRad(Ptest2.getGammaRad());

								smartServoLINRuntime.updateWithRealtimeSystem();
								smartServoLINRuntime.setDestination(destFrame);
							
							   }
							}

						}
						 catch (Exception e) {
								// TODO: handle exception
								System.out.println(e.toString());
							}
						smartServoLINRuntime.stopMotion();
					}
					
					else if (nToolMode==8 || nToolMode==9){
//						System.out.println("zuo");
						Frame current = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame());
						_toolAttachedToLBR_zuo_hand.move(ptp(current).setJointVelocityRel(0.1));

						SmartServoLIN aSmartServoLINMotion = new SmartServoLIN(current);
						aSmartServoLINMotion.setMinimumTrajectoryExecutionTime(20e-3);

						//设置是否有阻抗的地方
						//_toolAttachedToLBR.moveAsync(aSmartServoLINMotion.setMode(HardLimit_X()));
						_toolAttachedToLBR_zuo_hand.moveAsync(aSmartServoLINMotion);

						ISmartServoLINRuntime smartServoLINRuntime = aSmartServoLINMotion
								.getRuntime();

						px = py = pz = 0.03;
					
						pa = pb = pc=0.0003;
						

						Frame destFrame = current.copyWithRedundancy();
						
						ccx = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getX();
						ccy = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getY();
						ccz = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getZ();
						cca = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getAlphaRad();
						ccb = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getBetaRad();
						ccc = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getGammaRad();

						JointPosition jStart=lbr.getCurrentJointPosition();
						try {
							while (!bstop) {

								
								fx = lbr.getExternalForceTorque(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getForce()
										.getX();
								fy = lbr.getExternalForceTorque(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getForce()
										.getY();
								fz = lbr.getExternalForceTorque(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getForce()
										.getZ();
								fa = lbr.getExternalForceTorque(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getTorque().getZ();
								fb = lbr.getExternalForceTorque(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getTorque().getY();
								fc = lbr.getExternalForceTorque(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getTorque().getX();

					            cx = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getX();
								cy = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getY();
								cz = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getZ();
								ca = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getAlphaRad();
								cb = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getBetaRad();
								cc = lbr.getCurrentCartesianPosition(_toolAttachedToLBR_zuo_hand.getDefaultMotionFrame()).getGammaRad();

								
							 if (Math.abs(fy)>19 || Math.abs(fz)>19 || Math.abs(fc)>8){

								dfy = py * fy ;
								dfz = -pz * fz; 
								dfc =  pc * fc;


									JointPosition jNow=lbr.getCurrentJointPosition();
									double bBiaozhi=1;
									if (jNow.get(6)>0){
										bBiaozhi=1;
									}
									else{
										bBiaozhi=-1;
									}

										if(Math.abs(dfc) < 0.001){
											dfc = 0;					
										}
										
										if(Math.abs(Math.toDegrees(jStart.get(4))-Math.toDegrees(jNow.get(4)))>12 && (Math.toDegrees(jStart.get(4))-Math.toDegrees(jNow.get(4)))*fc*bBiaozhi<0  ){
											dfc = 0;	
										}
										
									
										if(Math.abs(ccy-cy)>100&& (ccy-cy)*fy<0 || Math.abs(dfy) < 0.001){
											dfy = 0;
										}
					
										if(Math.abs(ccz-cz)>75 && (ccz-cz)*fz>0 || Math.abs(dfz) < 0.001){
											dfz = 0;
										}



									if (Math.abs(dfz) < 0.5) {
										dfz = 0;
									}
									if (ccz - cz > 50) {
										if (fz > 0) {
											dfz = 0;

										}
									}

									if (ccz - cz < -50) {
										if (fz < 0) {
											dfz = 0;

										}

									}	
									
								Frame Ptest2 = destFrame.transform((Transformation.ofRad(0, dfy, -dfz, 0, 0, dfc)));
								destFrame.setX(Ptest2.getX());
								destFrame.setY(Ptest2.getY());
								destFrame.setZ(Ptest2.getZ());
								destFrame.setAlphaRad(Ptest2.getAlphaRad());
								destFrame.setBetaRad(Ptest2.getBetaRad());
								destFrame.setGammaRad(Ptest2.getGammaRad());
								
								smartServoLINRuntime.updateWithRealtimeSystem();
								smartServoLINRuntime.setDestination(destFrame);


//								// 获取最新的外部力信息
//								double[] externalForce =  smartServoLINRuntime.getExtForceMsr();


							 }
								
								
							}

						}
						 catch (Exception e) {
								
								System.out.println(e.toString());
							}
						smartServoLINRuntime.stopMotion();
					}
					
					

					


					ThreadUtil.milliSleep(20);
					nWorkingmode=0;
				}
				
				else if(nWorkingmode==10){
					//pingyi
					System.out.println("nWorkingmode==10");
					Frame cmdPos = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos.getX();
					cmdPos.getY();
					cmdPos.getZ();
					cmdPos.getAlphaRad();
					cmdPos.getBetaRad();
					cmdPos.getGammaRad();
					System.out.println("cmdPos:"+cmdPos);


					if (nTest==0 && nToolMode==1){
						jointPos_register =new JointPosition(   Math.toRadians(-26.68),
								Math.toRadians(-15.66),
								Math.toRadians(6.14),
								Math.toRadians(93.59),
								Math.toRadians(-23.73),
								Math.toRadians(56.22),
								Math.toRadians(25.43));	
					}
					else if (nTest==1 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-5.77),
								Math.toRadians(3.22),
								Math.toRadians(6.14),
								Math.toRadians(97.57),
								Math.toRadians(-1.42),
								Math.toRadians(37.47),
								Math.toRadians(2.57));	
					}
					else if (nTest==2 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-8.56),
								Math.toRadians(-16.90),
								Math.toRadians(6.14),
								Math.toRadians(98.93),
								Math.toRadians(-5.32),
								Math.toRadians(58.86),
								Math.toRadians(4.07));	
					}
					else if (nTest==3 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-29.52),
								Math.toRadians(-23.92),
								Math.toRadians(6.14),
								Math.toRadians(89.33),
								Math.toRadians(-26.11),
								Math.toRadians(60.82),
								Math.toRadians(26.32));	
					}
					else if (nTest==4 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(2.36),
								Math.toRadians(-18.98),
								Math.toRadians(6.14),
								Math.toRadians(84.23),
								Math.toRadians(6.06),
								Math.toRadians(46.66),
								Math.toRadians(-8.96));	
					}
					else if (nTest==5 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-26.03),
								Math.toRadians(-25.32),
								Math.toRadians(6.14),
								Math.toRadians(75.64),
								Math.toRadians(-26.98),
								Math.toRadians(48.48),
								Math.toRadians(29.32));	
					}
					///???
					else if (nTest==6 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-23.81),
								Math.toRadians(-34.16),
								Math.toRadians(6.14),
								Math.toRadians(73.27),
								Math.toRadians(-23.69),
								Math.toRadians(53.56),
								Math.toRadians(23.51));	
					}
					else if (nTest==7 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(4.42),
								Math.toRadians(-27.68),
								Math.toRadians(6.14),
								Math.toRadians(81.89),
								Math.toRadians(6.68),
								Math.toRadians(53.23),
								Math.toRadians(-10.47));	
					}
					else if (nTest==8 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-5.14),
								Math.toRadians(-7.64),
								Math.toRadians(6.14),
								Math.toRadians(90.72),
								Math.toRadians(-1.51),
								Math.toRadians(41.38),
								Math.toRadians(1.26));	
					}
					else if (nTest==9 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-7.65),
								Math.toRadians(-9.14),
								Math.toRadians(6.14),
								Math.toRadians(83.56),
								Math.toRadians(-5.46),
								Math.toRadians(35.83),
								Math.toRadians(5.82));	
					}
					else if (nTest==10 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-29.59),
								Math.toRadians(-16.83),
								Math.toRadians(6.14),
								Math.toRadians(74.29),
								Math.toRadians(-34.39),
								Math.toRadians(41.59),
								Math.toRadians(40.49));	
					}
					else if (nTest==11 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-43.70),
								Math.toRadians(0.53),
								Math.toRadians(6.14),
								Math.toRadians(111.20),
								Math.toRadians(-35.53),
								Math.toRadians(65.48),
								Math.toRadians(40.99));	
					}
					else if (nTest==12 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(1.71),
								Math.toRadians(-0.67),
								Math.toRadians(6.14),
								Math.toRadians(105.63),
								Math.toRadians(6.80),
								Math.toRadians(49.66),
								Math.toRadians(-7.40));	
					}
					else if (nTest==13 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(4.37),
								Math.toRadians(-12.49),
								Math.toRadians(6.14),
								Math.toRadians(105.17),
								Math.toRadians(7.54),
								Math.toRadians(61.27),
								Math.toRadians(-9.08));	
					}
					else if (nTest==14 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-13.85),
								Math.toRadians(-12.95),
								Math.toRadians(6.14),
								Math.toRadians(104.74),
								Math.toRadians(-9.91),
								Math.toRadians(61.21),
								Math.toRadians(9.32));	
					}
//					

					if (nTest==0 && nToolMode==7){
						jointPos_register =new JointPosition(   Math.toRadians(16.45),
								Math.toRadians(-24.52),
								Math.toRadians(-9.83),
								Math.toRadians(83.13),
								Math.toRadians(4.49),
								Math.toRadians(58.28),
								Math.toRadians(85.46));	
					}
					else if (nTest==1 && nToolMode==7) {
						jointPos_register =new JointPosition(   Math.toRadians(27.02),
								Math.toRadians(-26.18),
								Math.toRadians(-9.83),
								Math.toRadians(89.83),
								Math.toRadians(13.08),
								Math.toRadians(67.27),
								Math.toRadians(75.83));	
					}
					else if (nTest==2 && nToolMode==7) {
						jointPos_register =new JointPosition(   Math.toRadians(4.85),
								Math.toRadians(-20.60),
								Math.toRadians(-9.83),
								Math.toRadians(85.82),
								Math.toRadians(-6.47),
								Math.toRadians(58.08),
								Math.toRadians(98.59));	
					}
					else if (nTest==3 && nToolMode==7) {
						jointPos_register =new JointPosition(   Math.toRadians(25.47),
								Math.toRadians(-17.81),
								Math.toRadians(-9.83),
								Math.toRadians(99.31),
								Math.toRadians(10.35),
								Math.toRadians(68.30),
								Math.toRadians(77.47));	
					}
					else if (nTest==4 && nToolMode==7) {
						jointPos_register =new JointPosition(   Math.toRadians(-7.34),
								Math.toRadians(-16.93),
								Math.toRadians(-9.83),
								Math.toRadians(103.57),
								Math.toRadians(-15.54),
								Math.toRadians(74.71),
								Math.toRadians(107.44));	
					}
					else if (nTest==5 && nToolMode==7) {
						jointPos_register =new JointPosition(   Math.toRadians(18.94),
								Math.toRadians(-10.64),
								Math.toRadians(-9.83),
								Math.toRadians(103.92),
								Math.toRadians(4.06),
								Math.toRadians(65.45),
								Math.toRadians(83.24));	
					}
					///???
					else if (nTest==6 && nToolMode==7) {
						jointPos_register =new JointPosition(   Math.toRadians(7.22),
								Math.toRadians(-10.14),
								Math.toRadians(-9.83),
								Math.toRadians(107.16),
								Math.toRadians(-5.60),
								Math.toRadians(68.69),
								Math.toRadians(94.64));	
					}
					else if (nTest==7 && nToolMode==7) {
						jointPos_register =new JointPosition(   Math.toRadians(-9.48),
								Math.toRadians(-18.45),
								Math.toRadians(-9.83),
								Math.toRadians(100.60),
								Math.toRadians(-16.97),
								Math.toRadians(73.95),
								Math.toRadians(109.76));	
					}
					else if (nTest==8 && nToolMode==7) {
						jointPos_register =new JointPosition(   Math.toRadians(21.20),
								Math.toRadians(-22.84),
								Math.toRadians(-9.83),
								Math.toRadians(105.84),
								Math.toRadians(7.27),
								Math.toRadians(79.34),
								Math.toRadians(83.23));	
					}
					else if (nTest==9 && nToolMode==7) {
						jointPos_register =new JointPosition(   Math.toRadians(23.77),
								Math.toRadians(-8.55),
								Math.toRadians(-9.83),
								Math.toRadians(92.36),
								Math.toRadians(8.93),
								Math.toRadians(52.28),
								Math.toRadians(76.03));	
					}
					else if (nTest==10 && nToolMode==7) {
						jointPos_register =new JointPosition(   Math.toRadians(10.26),
								Math.toRadians(-5.24),
								Math.toRadians(-9.83),
								Math.toRadians(100.14),
								Math.toRadians(-4.27),
								Math.toRadians(56.56),
								Math.toRadians(92.38));	
					}
					else if (nTest==11 && nToolMode==7) {
						jointPos_register =new JointPosition(   Math.toRadians(16.82),
								Math.toRadians(-11.10),
								Math.toRadians(-9.83),
								Math.toRadians(99.14),
								Math.toRadians(2.45),
								Math.toRadians(61.09),
								Math.toRadians(85.19));	
					}
					else if (nTest==12 && nToolMode==7) {
						jointPos_register =new JointPosition(   Math.toRadians(30.97),
								Math.toRadians(-5.32),
								Math.toRadians(-9.83),
								Math.toRadians(106.71),
								Math.toRadians(13.37),
								Math.toRadians(64.47),
								Math.toRadians(70.46));	
					}
					else if (nTest==13 && nToolMode==7) {
						jointPos_register =new JointPosition(   Math.toRadians(5.40),
								Math.toRadians(-0.07),
								Math.toRadians(-9.83),
								Math.toRadians(108.56),
								Math.toRadians(-9.01),
								Math.toRadians(60.41),
								Math.toRadians(97.10));	
					}
					else if (nTest==14 && nToolMode==7) {
						jointPos_register =new JointPosition(   Math.toRadians(15.38),
								Math.toRadians(-9.63),
								Math.toRadians(-9.83),
								Math.toRadians(107.57),
								Math.toRadians(0.93),
								Math.toRadians(68.07),
								Math.toRadians(86.80));	
					}

					Err="10,";
					lbr.move(ptp(jointPos_register).setJointVelocityRel(0.2));
					//					System.out.println("move");
					Err="0,";
					nTest++;
					if (nTest==15){
						nTest=0;
					}
					//					System.out.println("nTest:"+nTest);
					nWorkingmode=0;
					//					System.out.println("nWorkingmode=0;");
				}
				else if(nWorkingmode==11){
					nLastWorkingmode=nWorkingmode;
					System.out.println("nWorkingmode=11");
					Frame Tcp = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					Tcp.setX(nX);
					Tcp.setY(nY);
					Tcp.setZ(nZ);
					Tcp.setAlphaRad(Math.toRadians(nA));
					Tcp.setBetaRad(Math.toRadians(nB));
					Tcp.setGammaRad(Math.toRadians(nC));


                    //Right saw blade verification point
					TRANSLATION_OF_TOOL_you3[0]=nX;
					TRANSLATION_OF_TOOL_you3[1]=nY;
					TRANSLATION_OF_TOOL_you3[2]=nZ;
					TRANSLATION_OF_TOOL_you3[3]=nA;
					TRANSLATION_OF_TOOL_you3[4]=nB;
					TRANSLATION_OF_TOOL_you3[5]=nC;
					

					//nToolMode==3   you_21003 上表面
					TRANSLATION_OF_TOOL_you2[0]=nX-10*Math.sin(nA);
					TRANSLATION_OF_TOOL_you2[1]=nY+10*Math.cos(nA);
					TRANSLATION_OF_TOOL_you2[2]=nZ;
					TRANSLATION_OF_TOOL_you2[3]=nA;
					TRANSLATION_OF_TOOL_you2[4]=nB;
					TRANSLATION_OF_TOOL_you2[5]=nC;
					
					
	
					double AA=Math.sin(nA);
					
					
					double nxs=10*Math.sin(nA);
					double nys=10*Math.cos(nA);
					double nxx=-10*Math.sin(nA)+1.8*Math.cos(nA);
					double nyx=10*Math.cos(nA)+1.8*Math.sin(nA);
					System.out.println("nxs"+nxs);
					System.out.println("nys"+nys);
                    System.out.println("nxx"+nxx);	
                    System.out.println("nyx"+nyx);
                    System.out.println("AA"+AA);
					
					System.out.println("TRANSLATION_OF_TOOL_you2[0]"+TRANSLATION_OF_TOOL_you2[0]);
					System.out.println("TRANSLATION_OF_TOOL_you2[1]"+TRANSLATION_OF_TOOL_you2[1]);
                    System.out.println("TRANSLATION_OF_TOOL_you2[2]"+TRANSLATION_OF_TOOL_you2[2]);
                    
                    
					//nToolMode==2   you_21002 下表面		
					TRANSLATION_OF_TOOL_you[0]=nX-10*Math.sin(nA)+1.8*Math.cos(nA);
					TRANSLATION_OF_TOOL_you[1]=nY+10*Math.cos(nA)+1.8*Math.sin(nA);
					TRANSLATION_OF_TOOL_you[2]=nZ;
					TRANSLATION_OF_TOOL_you[3]=nA;
					TRANSLATION_OF_TOOL_you[4]=nB;
					TRANSLATION_OF_TOOL_you[5]=nC;
					
					System.out.println("TRANSLATION_OF_TOOL_you[0]"+TRANSLATION_OF_TOOL_you[0]);
					System.out.println("TRANSLATION_OF_TOOL_you[1]"+TRANSLATION_OF_TOOL_you[1]);
                    System.out.println("TRANSLATION_OF_TOOL_you[2]"+TRANSLATION_OF_TOOL_you[2]);




					//Left saw blade verification point
					TRANSLATION_OF_TOOL_zuo3[0]=nX;
					TRANSLATION_OF_TOOL_zuo3[1]=nY;
					TRANSLATION_OF_TOOL_zuo3[2]=nZ;
					TRANSLATION_OF_TOOL_zuo3[3]=nA;
					TRANSLATION_OF_TOOL_zuo3[4]=nB;
					TRANSLATION_OF_TOOL_zuo3[5]=nC;
					
					//nToolMode==8     zuo_21002 下表面
					TRANSLATION_OF_TOOL_zuo[0]=nX-10;
					TRANSLATION_OF_TOOL_zuo[1]=nY-1.0;
					TRANSLATION_OF_TOOL_zuo[2]=nZ;
					TRANSLATION_OF_TOOL_zuo[3]=nA;
					TRANSLATION_OF_TOOL_zuo[4]=nB;
					TRANSLATION_OF_TOOL_zuo[5]=nC;

                    //nToolMode==9     zuo_21003 上表面
					TRANSLATION_OF_TOOL_zuo2[0]=nX-10;
					TRANSLATION_OF_TOOL_zuo2[1]=nY+0.8;
					TRANSLATION_OF_TOOL_zuo2[2]=nZ;
					TRANSLATION_OF_TOOL_zuo2[3]=nA;
					TRANSLATION_OF_TOOL_zuo2[4]=nB;
					TRANSLATION_OF_TOOL_zuo2[5]=nC;


//					TRANSLATION_OF_TOOL_zuo_hand[0]=nX;
//					TRANSLATION_OF_TOOL_zuo_hand[1]=nY;
//					TRANSLATION_OF_TOOL_zuo_hand[2]=nZ;
//					TRANSLATION_OF_TOOL_zuo_hand[3]=-1.5721;
//					TRANSLATION_OF_TOOL_zuo_hand[4]=3.1415;
//					TRANSLATION_OF_TOOL_zuo_hand[5]=0.0031;
					TRANSLATION_OF_TOOL_zuo_hand[3]=nA;
					TRANSLATION_OF_TOOL_zuo_hand[4]=nB;
					TRANSLATION_OF_TOOL_zuo_hand[5]=nC;
					
					
					
//					TRANSLATION_OF_TOOL_you_hand[0]=nX;
//					TRANSLATION_OF_TOOL_you_hand[1]=nY;
//					TRANSLATION_OF_TOOL_you_hand[2]=nZ;
					TRANSLATION_OF_TOOL_you_hand[3]=nA;
					TRANSLATION_OF_TOOL_you_hand[4]=nB;
					TRANSLATION_OF_TOOL_you_hand[5]=nC;

					nWorkingmode=0;
					System.out.println("11nWorkingmode=0");
				}
				else if(nWorkingmode==12){
					System.out.println("nWorkingmode=12");
					
					if (nTest_rotation==0 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-19.42),
								Math.toRadians(-10.57),
								Math.toRadians(6.14),
								Math.toRadians(97.40),
								Math.toRadians(-16.56),
								Math.toRadians(52.86),
								Math.toRadians(50.62));	
					}
//					else if (nTest_rotation==1 && nToolMode==1) {
//						jointPos_register =new JointPosition(   Math.toRadians(-3.30),
//								Math.toRadians(-17.11),
//								Math.toRadians(-6.13),
//								Math.toRadians(93.75),
//								Math.toRadians(-3.46),
//								Math.toRadians(53.55),
//								Math.toRadians(31.40));	
//					}
					
					else if (nTest_rotation==1 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-29.67),
								Math.toRadians(-28.40),
								Math.toRadians(6.14),
								Math.toRadians(99.53),
								Math.toRadians(-24.10),
								Math.toRadians(74.12),
								Math.toRadians(86.43));	
					}
					else if (nTest_rotation==2 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-30.77),
								Math.toRadians(-6.25),
								Math.toRadians(-6.13),
								Math.toRadians(113.05),
								Math.toRadians(-28.37),
								Math.toRadians(65.75),
								Math.toRadians(4.46));	
					}
					else if (nTest_rotation==3 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-18.32),
								Math.toRadians(-10.32),
								Math.toRadians(6.14),
								Math.toRadians(106.39),
								Math.toRadians(-13.95),
								Math.toRadians(61.06),
								Math.toRadians(-5.88));	
					}
					else if (nTest_rotation==4 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-42.33),
								Math.toRadians(-11.26),
								Math.toRadians(6.14),
								Math.toRadians(99.48),
								Math.toRadians(-35.63),
								Math.toRadians(64.56),
								Math.toRadians(-34.67));
					}
					else if (nTest_rotation==5 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-22.15),
								Math.toRadians(8.28),
								Math.toRadians(6.14),
								Math.toRadians(110.44),
								Math.toRadians(-19.21),
								Math.toRadians(48.38),
								Math.toRadians(-50.09));
					}
//					else if (nTest_rotation==6 && nToolMode==1) {
//						jointPos_register =new JointPosition(   Math.toRadians(8.17),
//								Math.toRadians(11.96),
//								Math.toRadians(-2.50),
//								Math.toRadians(108.03),
//								Math.toRadians(5.03),
//								Math.toRadians(39.36),
//								Math.toRadians(-138.94));
//					}
					else if (nTest_rotation==6 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(8.17),
								Math.toRadians(11.96),
								Math.toRadians(-2.50),
								Math.toRadians(108.03),
								Math.toRadians(5.03),
								Math.toRadians(39.36),
								Math.toRadians(-156.29));
					}
//					else if (nTest_rotation==7 && nToolMode==1) {
//						jointPos_register =new JointPosition(   Math.toRadians(11.59),
//								Math.toRadians(7.72),
//								Math.toRadians(-2.51),
//								Math.toRadians(104.03),
//								Math.toRadians(9.53),
//								Math.toRadians(39.98),
//								Math.toRadians(-142.96));
//					}
//					else if (nTest_rotation==7 && nToolMode==1) {
//						jointPos_register =new JointPosition(   Math.toRadians(-4.40),
//								Math.toRadians(15.44),
//								Math.toRadians(-2.51),
//								Math.toRadians(119.11),
//								Math.toRadians(-10.10),
//								Math.toRadians(47.35),
//								Math.toRadians(-174.21));
//					}
//					else if (nTest_rotation==8 && nToolMode==1) {
//						jointPos_register =new JointPosition(   Math.toRadians(-7.70),
//								Math.toRadians(5.40),
//								Math.toRadians(-2.50),
//								Math.toRadians(104.09),
//								Math.toRadians(-14.70),
//								Math.toRadians(43.20),
//								Math.toRadians(-128.50));
//					}
					else if (nTest_rotation==8 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-7.70),
								Math.toRadians(5.40),
								Math.toRadians(-2.50),
								Math.toRadians(104.09),
								Math.toRadians(-14.70),
								Math.toRadians(43.21),
								Math.toRadians(-71.57));
					}
//					else if (nTest_rotation==9 && nToolMode==1) {
//						jointPos_register =new JointPosition(   Math.toRadians(-5.55),
//								Math.toRadians(2.18),
//								Math.toRadians(-2.50),
//								Math.toRadians(105.53),
//								Math.toRadians(-11.10),
//								Math.toRadians(47.28),
//								Math.toRadians(-98.17));
//					}
					else if (nTest_rotation==9 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-5.54),
								Math.toRadians(2.18),
								Math.toRadians(-2.50),
								Math.toRadians(105.53),
								Math.toRadians(-11.10),
								Math.toRadians(47.28),
								Math.toRadians(-21.55));
					}
					else if (nTest_rotation==10 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-12.98),
								Math.toRadians(-8.50),
								Math.toRadians(-2.50),
								Math.toRadians(98.70),
								Math.toRadians(-17.70),
								Math.toRadians(52.84),
								Math.toRadians(-19.46));
					}
					else if (nTest_rotation==11 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-19.82),
								Math.toRadians(-18.80),
								Math.toRadians(-2.50),
								Math.toRadians(101.96),
								Math.toRadians(-20.91),
								Math.toRadians(67.77),
								Math.toRadians(7.91));
					}
					else if (nTest_rotation==12 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-6.34),
								Math.toRadians(-11.70),
								Math.toRadians(-2.50),
								Math.toRadians(107.93),
								Math.toRadians(-9.41),
								Math.toRadians(63.43),
								Math.toRadians(33.88));
					}
					else if (nTest_rotation==13 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(-6.63),
								Math.toRadians(-10.61),
								Math.toRadians(-2.50),
								Math.toRadians(108.04),
								Math.toRadians(-9.81),
								Math.toRadians(62.52),
								Math.toRadians(3.70));
					}
					else if (nTest_rotation==14 && nToolMode==1) {
						jointPos_register =new JointPosition(   Math.toRadians(7.01),
								Math.toRadians(-19.00),
								Math.toRadians(-2.51),
								Math.toRadians(90.33),
								Math.toRadians(3.80),
								Math.toRadians(52.43),
								Math.toRadians(12.52));
					}
					///
					
					
					
					else if (nTest_rotation==0 && nToolMode==7) {
					jointPos_register =new JointPosition(   Math.toRadians(-7.48),
							Math.toRadians(-24.21),
							Math.toRadians(8.81),
							Math.toRadians(85.78),
							Math.toRadians(-6.64),
							Math.toRadians(62.22),
							Math.toRadians(61.61));	
				}
				    else if (nTest_rotation==1 && nToolMode==7) {
					jointPos_register =new JointPosition(   Math.toRadians(-10.56),
							Math.toRadians(-5.60),
							Math.toRadians(8.81),
							Math.toRadians(102.69),
							Math.toRadians(-6.59),
							Math.toRadians(60.96),
							Math.toRadians(13.05));
				}
				    else if (nTest_rotation==2 && nToolMode==7) {
					jointPos_register =new JointPosition(   Math.toRadians(-4.18),
							Math.toRadians(-15.38),
							Math.toRadians(8.81),
							Math.toRadians(91.86),
							Math.toRadians(-2.58),
							Math.toRadians(59.57),
							Math.toRadians(-60.36));
				}
				    else if (nTest_rotation==3 && nToolMode==7) {
					jointPos_register =new JointPosition(   Math.toRadians(-4.18),
							Math.toRadians(-21.57),
							Math.toRadians(8.80),
							Math.toRadians(92.46),
							Math.toRadians(-3.37),
							Math.toRadians(66.18),
							Math.toRadians(69.24));	
				}

				   else if (nTest_rotation==4 && nToolMode==7) {
					jointPos_register =new JointPosition(   Math.toRadians(17.94),
							Math.toRadians(-23.82),
							Math.toRadians(8.80),
							Math.toRadians(87.00),
							Math.toRadians(13.89),
							Math.toRadians(66.15),
							Math.toRadians(-172.83));	
				}
				   else if (nTest_rotation==5 && nToolMode==7) {
					jointPos_register =new JointPosition(   Math.toRadians(8.12),
							Math.toRadians(-4.84),
							Math.toRadians(8.80),
							Math.toRadians(107.85),
							Math.toRadians(8.83),
							Math.toRadians(65.99),
							Math.toRadians(-0.82));	
				}
					//
				    else if (nTest_rotation==6 && nToolMode==7) {
				    jointPos_register =new JointPosition(   Math.toRadians(40.89),
						Math.toRadians(-17.35),
						Math.toRadians(-19.71),
						Math.toRadians(84.93),
						Math.toRadians(25.21),
						Math.toRadians(48.86),
						Math.toRadians(-143.55));	
				}

				   else if (nTest_rotation==7 && nToolMode==7) {
					jointPos_register =new JointPosition(   Math.toRadians(11.03),
							Math.toRadians(-18.34),
							Math.toRadians(8.82),
							Math.toRadians(103.20),
							Math.toRadians(8.80),
							Math.toRadians(75.13),
							Math.toRadians(129.64));	
				}
				   else if (nTest_rotation==8 && nToolMode==7) {
					jointPos_register =new JointPosition(   Math.toRadians(-4.44),
							Math.toRadians(-5.32),
							Math.toRadians(8.83),
							Math.toRadians(99.77),
							Math.toRadians(-1.44),
							Math.toRadians(57.44),
							Math.toRadians(-112.51));	
				}
				   else if (nTest_rotation==9 && nToolMode==7) {
					jointPos_register =new JointPosition(   Math.toRadians(23.26),
						Math.toRadians(0.55),
						Math.toRadians(-19.17),
						Math.toRadians(111.23),
						Math.toRadians(0.29),
						Math.toRadians(55.31),
						Math.toRadians(-26.38));
				}
				   else if (nTest_rotation==10 && nToolMode==7) {
					jointPos_register =new JointPosition(   Math.toRadians(30.28),
							Math.toRadians(-19.30),
							Math.toRadians(8.80),
							Math.toRadians(96.13),
							Math.toRadians(22.64),
							Math.toRadians(74.81),
							Math.toRadians(-174.30));
				}
				   else if (nTest_rotation==11 && nToolMode==7) {
					   jointPos_register =new JointPosition(   Math.toRadians(42.38),
								Math.toRadians(-17.67),
								Math.toRadians(-19.71),
								Math.toRadians(88.61),
								Math.toRadians(25.40),
								Math.toRadians(52.91),
								Math.toRadians(-105.36));	
				}
				   else if (nTest_rotation==12 && nToolMode==7) {
					jointPos_register =new JointPosition(   Math.toRadians(16.98),
						Math.toRadians(8.14),
						Math.toRadians(-19.71),
						Math.toRadians(102.05),
						Math.toRadians(-9.98),
						Math.toRadians(39.36),
						Math.toRadians(-43.35));
				}
				    else if (nTest_rotation==13 && nToolMode==7) {
					jointPos_register =new JointPosition(   Math.toRadians(3.20),
							Math.toRadians(-22.57),
							Math.toRadians(8.81),
							Math.toRadians(92.19),
							Math.toRadians(2.40),
							Math.toRadians(67.25),
							Math.toRadians(65.06));	
				}
					   else if (nTest_rotation==14 && nToolMode==7) {
						jointPos_register =new JointPosition(   Math.toRadians(29.01),
								Math.toRadians(-16.13),
								Math.toRadians(8.80),
								Math.toRadians(94.78),
								Math.toRadians(29.69),
								Math.toRadians(75.27),
								Math.toRadians(-98.16));
					}
					 Err="10,";
						lbr.move(ptp(jointPos_register).setJointVelocityRel(0.2));
						Err="0,";
						nTest_rotation++;
						if (nTest_rotation==15){
							nTest_rotation=0;
						}
						nWorkingmode=0;
				}
				else if(nWorkingmode==13){
					nLastWorkingmode=nWorkingmode;
					if(n_Test==1){
					jointPos_zuo=new JointPosition(   Math.toRadians(-2.85),
			                Math.toRadians(-54.36),
			                Math.toRadians(-32.39),
			                Math.toRadians(29.48),
			                Math.toRadians(89.76),
			                Math.toRadians(-24.98),
			                Math.toRadians(146.98));
					}
					else if(n_Test==2){
						jointPos_zuo=new JointPosition(   Math.toRadians(61.27),
				                Math.toRadians(-38.39),
				                Math.toRadians(115.98),
				                Math.toRadians(-83.10),
				                Math.toRadians(55.26),
				                Math.toRadians(-10.35),
				                Math.toRadians(0.46));
					}
					else if(n_Test==3){
						jointPos_zuo=new JointPosition(   Math.toRadians(28.63),
				                Math.toRadians(-37.68),
				                Math.toRadians(115.56),
				                Math.toRadians(-65.59),
				                Math.toRadians(62.02),
				                Math.toRadians(9.03),
				                Math.toRadians(0.47));
					}
					else if(n_Test==4){
						jointPos_zuo=new JointPosition(   Math.toRadians(-45.18),
				                Math.toRadians(-36.79),
				                Math.toRadians(83.72),
				                Math.toRadians(73.94),
				                Math.toRadians(-89.93),
				                Math.toRadians(-20.83),
				                Math.toRadians(-127.41));
					}
					else if(n_Test==5){
						jointPos_zuo=new JointPosition(   Math.toRadians(69.18),
				                Math.toRadians(-68.03),
				                Math.toRadians(66.64),
				                Math.toRadians(-81.61),
				                Math.toRadians(66.14),
				                Math.toRadians(26.67),
				                Math.toRadians(3.95));
					}
					else if(n_Test==6){
						jointPos_zuo=new JointPosition(   Math.toRadians(-39.47),
				                Math.toRadians(-59.68),
				                Math.toRadians(-111.68),
				                Math.toRadians(-45.79),
				                Math.toRadians(-40.64),
				                Math.toRadians(7.82),
				                Math.toRadians(2.69));
					}
					else if(n_Test==7){
						jointPos_zuo=new JointPosition(   Math.toRadians(-55.36),
				                Math.toRadians(-32.76),
				                Math.toRadians(-66.13),
				                Math.toRadians(-68.97),
				                Math.toRadians(90.08),
				                Math.toRadians(-55.26),
				                Math.toRadians(-119.37));
					}
					else if(n_Test==8){
						jointPos_zuo=new JointPosition(   Math.toRadians(34.60),
				                Math.toRadians(-45.98),
				                Math.toRadians(-20.16),
				                Math.toRadians(54.48),
				                Math.toRadians(-88.00),
				                Math.toRadians(-34.85),
				                Math.toRadians(-53.77));
					}
					else if(n_Test==9){
						jointPos_zuo=new JointPosition(   Math.toRadians(69.92),
				                Math.toRadians(-28.50),
				                Math.toRadians(78.15),
				                Math.toRadians(-55.29),
				                Math.toRadians(54.39),
				                Math.toRadians(45.21),
				                Math.toRadians(6.95));
					}
					else if(n_Test==10){
						jointPos_zuo=new JointPosition(   Math.toRadians(-16.51),
				                Math.toRadians(-41.03),
				                Math.toRadians(28.38),
				                Math.toRadians(59.33),
				                Math.toRadians(90.05),
				                Math.toRadians(-17.36),
				                Math.toRadians(79.03));
					}
					else if(n_Test==11){
						jointPos_zuo=new JointPosition(   Math.toRadians(59.64),
				                Math.toRadians(-8.84),
				                Math.toRadians(-31.64),
				                Math.toRadians(79.57),
				                Math.toRadians(-78.50),
				                Math.toRadians(-57.02),
				                Math.toRadians(-77.02));
					}
					else if(n_Test==12){
						jointPos_zuo=new JointPosition(   Math.toRadians(13.07),
				               Math.toRadians(-39.25),
				                Math.toRadians(7.50),
				                Math.toRadians(47.68),
				                Math.toRadians(-88.19),
				                Math.toRadians(-47.39),
				                Math.toRadians(-87.12));
					}
					else if(n_Test==13){
						jointPos_zuo=new JointPosition(   Math.toRadians(-40.67),
				                Math.toRadians(-38.44),
				                Math.toRadians(-116.80),
				                Math.toRadians(-87.81),
				                Math.toRadians(-23.29),
				                Math.toRadians(-15.04),
				                Math.toRadians(0.61));
					}
					else if(n_Test==14){
						jointPos_zuo=new JointPosition(   Math.toRadians(-29.96),
				                Math.toRadians(-13.80),
				                Math.toRadians(33.48),
				                Math.toRadians(79.96),
				                Math.toRadians(-92.34),
				                Math.toRadians(-24.33),
				                Math.toRadians(-80.81));
					}
					else if(n_Test==15){
						jointPos_zuo=new JointPosition(   Math.toRadians(-65.71),
				                Math.toRadians(-51.77),
				                Math.toRadians(-98.71),
				                Math.toRadians(-89.03),
				                Math.toRadians(-39.51),
				                Math.toRadians(-6.91),
				                Math.toRadians(1.28));
					}
					else if(n_Test==16){
						jointPos_zuo=new JointPosition(   Math.toRadians(33.51),
				                Math.toRadians(-47.93),
				                Math.toRadians(-15.93),
				                Math.toRadians(55.20),
				                Math.toRadians(89.99),
				                Math.toRadians(48.27),
				                Math.toRadians(122.80));
					}
					else if(n_Test==17){
						jointPos_zuo=new JointPosition(   Math.toRadians(-6.68),
				                Math.toRadians(-33.05),
				                Math.toRadians(-36.48),
				                Math.toRadians(50.17),
				                Math.toRadians(89.21),
				                Math.toRadians(-42.10),
				                Math.toRadians(130.60));
					}
					else if(n_Test==18){
						jointPos_zuo=new JointPosition(   Math.toRadians(-8.65),
				                Math.toRadians(-39.87),
				                Math.toRadians(95.22),
				                Math.toRadians(24.82),
				                Math.toRadians(108.46),
				                Math.toRadians(54.49),
				                Math.toRadians(10.04));
					}
					else if(n_Test==19){
						jointPos_zuo=new JointPosition(   Math.toRadians(-36.38),
				                Math.toRadians(-64.73),
				                Math.toRadians(56.64),
				                Math.toRadians(62.86),
				                Math.toRadians(90.37),
				                Math.toRadians(-22.93),
				                Math.toRadians(57.42));
					}
					else if(n_Test==20){
						jointPos_zuo=new JointPosition(   Math.toRadians(53.97),
				                Math.toRadians(-51.54),
				                Math.toRadians(100.91),
				                Math.toRadians(-82.30),
				                Math.toRadians(90.01),
				                Math.toRadians(-4.85),
				                Math.toRadians(-19.04));
					}
					else if(n_Test==21){
						jointPos_zuo=new JointPosition(   Math.toRadians(-73.18),
				                Math.toRadians(-42.47),
				                Math.toRadians(-70.50),
				                Math.toRadians(-64.34),
				                Math.toRadians(-52.23),
				                Math.toRadians(37.68),
				                Math.toRadians(4.76));
					}
					else if(n_Test==22){
						jointPos_zuo=new JointPosition(   Math.toRadians(13.84),
				                Math.toRadians(-48.40),
				                Math.toRadians(-62.70),
				                Math.toRadians(48.58),
				                Math.toRadians(-117.63),
				                Math.toRadians(13.97),
				                Math.toRadians(0.23));
					}
					else if(n_Test==23){
						jointPos_zuo=new JointPosition(   Math.toRadians(31.66),
				                Math.toRadians(-34.44),
				                Math.toRadians(72.13),
				                Math.toRadians(-56.25),
				                Math.toRadians(-90.04),
				                Math.toRadians(-57.52),
				                Math.toRadians(143.57));
					}
					else if(n_Test==24){
						jointPos_zuo=new JointPosition(   Math.toRadians(34.46),
				                Math.toRadians(-55.25),
				                Math.toRadians(-50.94),
				                Math.toRadians(59.08),
				                Math.toRadians(-90.69),
				                Math.toRadians(-15.41),
				                Math.toRadians(-33.65));
					}
					else if(n_Test==25){
						jointPos_zuo=new JointPosition(   Math.toRadians(28.11),
				                Math.toRadians(-62.29),
				                Math.toRadians(-80.33),
				                Math.toRadians(46.38),
				                Math.toRadians(-75.42),
				                Math.toRadians(17.49),
				                Math.toRadians(-10.28));
					}
					else if(n_Test==26){
						jointPos_zuo=new JointPosition(   Math.toRadians(40.49),
				                Math.toRadians(-85.72),
				                Math.toRadians(54.76),
				                Math.toRadians(-58.65),
				                Math.toRadians(87.36),
				                Math.toRadians(33.53),
				                Math.toRadians(28.31));
						}
						else if(n_Test==27){
							jointPos_zuo=new JointPosition(   Math.toRadians(-35.66),
					                Math.toRadians(-34.73),
					                Math.toRadians(73.50),
					                Math.toRadians(47.37),
					                Math.toRadians(-89.56),
					                Math.toRadians(-47.52),
					                Math.toRadians(-131.05));
						}
						else if(n_Test==28){
							jointPos_zuo=new JointPosition(   Math.toRadians(-101.48),
					                Math.toRadians(41.51),
					                Math.toRadians(46.84),
					                Math.toRadians(71.31),
					                Math.toRadians(-109.25),
					                Math.toRadians(53.92),
					                Math.toRadians(-0.59));
						}
						else if(n_Test==29){
							jointPos_zuo=new JointPosition(   Math.toRadians(-20.41),
					                Math.toRadians(-55.39),
					                Math.toRadians(89.22),
					                Math.toRadians(39.13),
					                Math.toRadians(-89.96),
					                Math.toRadians(-32.24),
					                Math.toRadians(-140.62));
						}
						else if(n_Test==30){
							jointPos_zuo=new JointPosition(   Math.toRadians(5.68),
					                Math.toRadians(-51.85),
					                Math.toRadians(35.05),
					                Math.toRadians(30.54),
					                Math.toRadians(-89.02),
					                Math.toRadians(-28.63),
					                Math.toRadians(-101.39));
						}
						else if(n_Test==31){
							jointPos_zuo=new JointPosition(   Math.toRadians(47.51),
					                Math.toRadians(-52.12),
					                Math.toRadians(116.07),
					                Math.toRadians(-76.26),
					                Math.toRadians(53.97),
					                Math.toRadians(-12.52),
					                Math.toRadians(0.36));
						}
						else if(n_Test==32){
							jointPos_zuo=new JointPosition(   Math.toRadians(10.86),
					                Math.toRadians(-42.24),
					                Math.toRadians(-50.84),
					                Math.toRadians(29.72),
					                Math.toRadians(-89.97),
					                Math.toRadians(48.25),
					                Math.toRadians(-43.11));
						}
						else if(n_Test==33){
							jointPos_zuo=new JointPosition(   Math.toRadians(-51.17),
					                Math.toRadians(-19.89),
					                Math.toRadians(29.95),
					                Math.toRadians(84.80),
					                Math.toRadians(87.14),
					                Math.toRadians(-53.19),
					                Math.toRadians(95.84));
						}
						else if(n_Test==34){
							jointPos_zuo=new JointPosition(   Math.toRadians(-43.91),
					                Math.toRadians(-37.87),
					                Math.toRadians(31.97),
					                Math.toRadians(77.71),
					                Math.toRadians(93.02),
					                Math.toRadians(-44.63),
					                Math.toRadians(66.41));
						}
						else if(n_Test==35){
							jointPos_zuo=new JointPosition(   Math.toRadians(70.16),
					                Math.toRadians(-18.03),
					                Math.toRadians(-65.32),
					                Math.toRadians(82.97),
					                Math.toRadians(89.59),
					                Math.toRadians(-9.61),
					                Math.toRadians(122.69));
						}
						else if(n_Test==36){
							jointPos_zuo=new JointPosition(   Math.toRadians(40.88),
					                Math.toRadians(-76.70),
					                Math.toRadians(-85.13),
					                Math.toRadians(45.15),
					                Math.toRadians(-89.70),
					                Math.toRadians(3.52),
					                Math.toRadians(12.29));
						}
						else if(n_Test==37){
							jointPos_zuo=new JointPosition(   Math.toRadians(15.54),
					               Math.toRadians(-33.22),
					                Math.toRadians(9.79),
					                Math.toRadians(56.15),
					                Math.toRadians(-88.62),
					                Math.toRadians(-33.92),
					                Math.toRadians(-82.46));
						}
						else if(n_Test==38){
							jointPos_zuo=new JointPosition(   Math.toRadians(0.39),
					                Math.toRadians(-48.25),
					                Math.toRadians(59.73),
					                Math.toRadians(9.01),
					                Math.toRadians(-76.30),
					                Math.toRadians(-46.03),
					                Math.toRadians(-149.47));
						}
						else if(n_Test==39){
							jointPos_zuo=new JointPosition(   Math.toRadians(59.99),
					                Math.toRadians(-33.48),
					                Math.toRadians(-109.01),
					                Math.toRadians(58.85),
					                Math.toRadians(-99.86),
					                Math.toRadians(52.15),
					                Math.toRadians(-4.04));
						}
						else if(n_Test==40){
							jointPos_zuo=new JointPosition(   Math.toRadians(48.51),
					                Math.toRadians(-52.07),
					                Math.toRadians(-114.84),
					                Math.toRadians(43.92),
					                Math.toRadians(-112.90),
					                Math.toRadians(49.10),
					                Math.toRadians(12.11));
						}
						else if(n_Test==41){
							jointPos_zuo=new JointPosition(   Math.toRadians(-42.28),
					                Math.toRadians(-85.27),
					                Math.toRadians(-69.92),
					                Math.toRadians(-57.15),
					                Math.toRadians(-86.49),
					                Math.toRadians(17.92),
					                Math.toRadians(9.83));
						}
						else if(n_Test==42){
							jointPos_zuo=new JointPosition(   Math.toRadians(5.27),
					                Math.toRadians(-56.44),
					                Math.toRadians(-73.39),
					                Math.toRadians(28.88),
					                Math.toRadians(-107.30),
					                Math.toRadians(25.69),
					                Math.toRadians(15.11));
						}
						else if(n_Test==43){
							jointPos_zuo=new JointPosition(   Math.toRadians(-61.52),
					                Math.toRadians(-25.73),
					                Math.toRadians(-111.95),
					                Math.toRadians(-63.45),
					                Math.toRadians(-5.56),
					                Math.toRadians(21.46),
					                Math.toRadians(2.72));
						}
						else if(n_Test==44){
							jointPos_zuo=new JointPosition(   Math.toRadians(-5.42),
					                Math.toRadians(-22.20),
					                Math.toRadians(39.31),
					                Math.toRadians(70.75),
					                Math.toRadians(-90.67),
					                Math.toRadians(-23.31),
					                Math.toRadians(-98.91));
						}
						else if(n_Test==45){
							jointPos_zuo=new JointPosition(   Math.toRadians(23.68),
					                Math.toRadians(-47.13),
					                Math.toRadians(-6.70),
					                Math.toRadians(45.96),
					                Math.toRadians(-88.68),
					                Math.toRadians(-30.95),
					                Math.toRadians(-70.36));
						}
						else if(n_Test==46){
							jointPos_zuo=new JointPosition(   Math.toRadians(-43.49),
					                Math.toRadians(-28.76),
					                Math.toRadians(41.75),
					                Math.toRadians(74.95),
					                Math.toRadians(90.22),
					                Math.toRadians(-14.76),
					                Math.toRadians(70.17));
						}
						else if(n_Test==47){
							jointPos_zuo=new JointPosition(   Math.toRadians(-48.43),
					                Math.toRadians(-55.42),
					                Math.toRadians(-119.05),
					                Math.toRadians(-64.24),
					                Math.toRadians(-24.36),
					                Math.toRadians(-8.23),
					                Math.toRadians(0.66));
						}
						else if(n_Test==48){
							jointPos_zuo=new JointPosition(   Math.toRadians(-70.84),
					                Math.toRadians(-35.35),
					                Math.toRadians(-91.15),
					                Math.toRadians(-53.29),
					                Math.toRadians(-26.12),
					                Math.toRadians(36.04),
					                Math.toRadians(-9.41));
						}
						else if(n_Test==49){
							jointPos_zuo=new JointPosition(   Math.toRadians(36.59),
					                Math.toRadians(-69.44),
					                Math.toRadians(-58.20),
					                Math.toRadians(40.26),
					                Math.toRadians(-86.26),
					                Math.toRadians(-7.57),
					                Math.toRadians(-10.34));
						}
						else if(n_Test==50){
							jointPos_zuo=new JointPosition(   Math.toRadians(41.74),
					                Math.toRadians(-70.21),
					                Math.toRadians(47.11),
					                Math.toRadians(-55.41),
					                Math.toRadians(85.06),
					                Math.toRadians(51.05),
					                Math.toRadians(28.69));
						}
					lbr.move(ptp(jointPos_zuo).setJointVelocityRel(0.1));
//					nTest++;
					nWorkingmode=0;
				}
				else if (nWorkingmode==14){
					nLastWorkingmode=nWorkingmode;
					n_Test++;
					System.out.println("n_Test:"+n_Test);
					if(n_Test==51){
						n_Test=0;
					}
					nWorkingmode=0;
				}
				else if(nWorkingmode==15){
					bstop=true;
					nLastWorkingmode=nWorkingmode;
					
					Frame pre_Place1 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
					pre_Place1.setX(nX);
					pre_Place1.setY(nY);
					pre_Place1.setZ(nZ);
					pre_Place1.setAlphaRad(Math.toRadians(nA));
					pre_Place1.setBetaRad(Math.toRadians(nB));
					pre_Place1.setGammaRad(Math.toRadians(nC));	
					getLogger().info("目标点pre_Place1:"+pre_Place1);

					
					

					pre_Place1 = pre_Place1.transform((Transformation.ofDeg(0,0,0, -A, -B, -C)));
					pre_Place1 = pre_Place1.transform((Transformation.ofDeg(-TRANSLATION_OF_TOOL_you[0] ,-TRANSLATION_OF_TOOL_you[1],-TRANSLATION_OF_TOOL_you[2], 0, 0, 0)));
					getLogger().info("变换后pre_Place1:"+pre_Place1);
					//pre_Place1 = pre_Place1.transform((Transformation.ofDeg(-TRANSLATION_OF_TOOL_you[0] ,-TRANSLATION_OF_TOOL_you[1],-TRANSLATION_OF_TOOL_you[2], 0, 0, 0)));
					
					
					JointPosition test1 = lbr.getInverseKinematicFromFrameAndRedundancy(pre_Place1);
					getLogger().info("joint4:"+test1.get(JointEnum.J4)*180/Math.PI);
					


					Frame pre_Place2 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();

					pre_Place2.setX(nX);
					pre_Place2.setY(nY);
					pre_Place2.setZ(nZ);
					pre_Place2.setAlphaRad(Math.toRadians(nA));
					pre_Place2.setBetaRad(Math.toRadians(nB));
					pre_Place2.setGammaRad(Math.toRadians(nC));
					
					try{
						if(nToolMode==3){
							
							_toolAttachedToLBR_you2.getDefaultMotionFrame().move(lin(pre_Place2).setJointVelocityRel(0.1).breakWhen(VaccumDetect));
							
										    		    	
						}
						else if(nToolMode==2){
					 
							_toolAttachedToLBR_you.getDefaultMotionFrame().move(lin(pre_Place2).setJointVelocityRel(0.1).breakWhen(VaccumDetect));							
						
							
						}
						else if(nToolMode==9){

							
							_toolAttachedToLBR_zuo2.getDefaultMotionFrame().move(lin(pre_Place2).setJointVelocityRel(0.1).breakWhen(VaccumDetect));
							
							
						}
						else if(nToolMode==8){

							_toolAttachedToLBR_zuo.getDefaultMotionFrame().move(lin(pre_Place2).setJointVelocityRel(0.1).breakWhen(VaccumDetect));

						}
					}
					catch(Throwable cause){
						cause.printStackTrace();
						getLogger().info("RangLimit:"+cause);
					}
					StartOnece=true;
					nWorkingmode=0;
					bstop=true;

						
				     
					}
				else{

									
					if(DangerMove==false && nLastWorkingmode==4){
						JointPosition jReady =lbr.getCurrentJointPosition();


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
								System.out.println("J4:"+jReady.get(JointEnum.J4)+"   MAX:115;MIN:-10");
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
							Frame Ptest1 = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/you_21001"));
							needle_Tool_2.getFrame("/you_21001").move(lin(Ptest1).setJointVelocityRel(0.2));
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

	public  class BreakTestProgram implements Callable<String> {


		public String call() {
			while (true){
				if(SafeDataIO.getInput4()==true){
					bstop = true;
				}
				if(SafeDataIO.getInput4()==false){
					bstop = false;
				}

				if(SafeDataIO.getInput4()==true && breaktest==true){
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
					lbr.move(new PTP(jointPos_zuo).setJointVelocityRel(0.2));
					//				  Err="100";
					breaktest=false;
				}
				ThreadUtil.milliSleep(500);
				//			System.out.println("new thread");

			}
			//return Err;	

		}
	}

	//ä¿¡å�·é‡‡é›†
	public void Monitor() {

		while (true)
		{



			try{
				ThreadUtil.milliSleep(5000);




				//æš‚æ—¶æ— æ„�ä¹‰ï¼ˆé¢„ç•™é»˜è®¤ä¸º0ï¼‰
				data0 = "$0,";

				//æŠ¥è­¦ä»£ç �
				data1 = "0,";

				//å·¥ä½œæ¨¡å¼�
				data2 = "0,";

				//æ˜¯å�¦ä¸Šç”µ
				data3 = "1,";

				//å½“å‰�é€Ÿåº¦
				//CartesianVelocityLimitInfo infoObject = lbr.getCartesianVelocityLimitInfo();
				double a1 =getApplicationControl().getApplicationOverride();
				BigDecimal bigDecimal0 = new BigDecimal(a1);
				a1 = bigDecimal0.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data4 = String.valueOf(a1)+",";

				//æ— æ„�ä¹‰
				data5 = "0,";

				//				//å…³èŠ‚åº¦æ•°1
				JointPosition actPos = lbr.getCurrentJointPosition();
				a1 = Math.toDegrees(actPos.get(0));
				BigDecimal bigDecimal = new BigDecimal(a1);
				a1 = bigDecimal.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data6=String.valueOf(a1)+",";

				//				//å…³èŠ‚åº¦æ•°2
				a1 = Math.toDegrees(actPos.get(1));
				BigDecimal bigDecimal1 = new BigDecimal(a1);
				a1 = bigDecimal1.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data7=String.valueOf(a1)+",";

				//				//å…³èŠ‚åº¦æ•°3
				a1 = Math.toDegrees(actPos.get(2));
				BigDecimal bigDecimal2 = new BigDecimal(a1);
				a1 = bigDecimal2.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data8=String.valueOf(a1)+",";

				//å…³èŠ‚åº¦æ•°4
				a1 = Math.toDegrees(actPos.get(3));
				BigDecimal bigDecimal3 = new BigDecimal(a1);
				a1 = bigDecimal3.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data9=String.valueOf(a1)+",";

				//å…³èŠ‚åº¦æ•°5
				a1 = Math.toDegrees(actPos.get(4));
				BigDecimal bigDecimal4 = new BigDecimal(a1);
				a1 = bigDecimal4.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data10=String.valueOf(a1)+",";

				//å…³èŠ‚åº¦æ•°6
				a1 = Math.toDegrees(actPos.get(5));
				BigDecimal bigDecimal5 = new BigDecimal(a1);
				a1 = bigDecimal5.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data11=String.valueOf(a1)+",";

				//å…³èŠ‚åº¦æ•°7
				a1 = Math.toDegrees(actPos.get(6));
				BigDecimal bigDecimal6 = new BigDecimal(a1);
				a1 = bigDecimal6.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data12=String.valueOf(a1)+",";

				//è½´å��æ ‡x
				Frame cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/you_21001"));
				cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/you_21001"));
				System.out.println("1:"+cmdPos);
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCommandedCartesianPosition(needle_Tool_2.getFrame("/you_21001"), cmdPos2);
				System.out.println("2:"+cmdPos);

				Frame cmdPos1 = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/CoverScrewing"));
				System.out.println("getFlange()"+cmdPos1);



				//				if (nToolMode==2){
				//					 cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_anfang"));
				//				}
				//				else{
				//					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/you_21001"));
				//				}

				if (nToolMode==1){
					cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/you_21001"));
					System.out.println("1:"+cmdPos);
					cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCommandedCartesianPosition(needle_Tool_2.getFrame("/you_21001"), cmdPos2);
					System.out.println("2:"+cmdPos);


				}
				else if(nToolMode==2)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/you_21002"));
				}	
				else if(nToolMode==3)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/you_21003"));
				}	
				else if(nToolMode==4)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/tcp_x_1_yz3"));
				}	
				else if(nToolMode==5)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/you_21004"));
				}	
				else if(nToolMode==6)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/you_21005"));
				}	
				else if(nToolMode==7)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_3.getFrame("/zuo_21001"));
				}	
				else if(nToolMode==8)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_3.getFrame("/zuo_21002"));
				}	
				else if(nToolMode==9)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_3.getFrame("/zuo_21003"));
				}	
				else if(nToolMode==10)
				{

					cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_3.getFrame("/zuo_21004"));
				}	
				else if(nToolMode==11)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_3.getFrame("/zuo_21005"));
				}	

				else
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle_Tool_2.getFrame("/test"));
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
			}catch(Exception ex){
				break;
			}
		}

	}

	public boolean MotionType() {
		bForPlane=SafeDataIO.getInput4();
		//		ThreadUtil.milliSleep(1000);
		//		System.out.println("tttplane");
		return bForPlane;
	}


	//@SuppressWarnings("null")
	@Override
	public void run()  {

		//	    io.setOutput3(true);
		//		BreakTest.initialize();
		//		BreakTest.run();
		////		lbr.moveAsync(new PTP(jointPos_zuo).setJointVelocityRel(0.2));
		//		ThreadUtil.milliSleep(2000000);

		//		OnlyPlane.run();
		//		ThreadUtil.milliSleep(2000);
		//		OnlyPlane.initialize();
		//		OnlyPlane.run();
		//////
		//		_loadData3 = new LoadData();
		//		  _loadData3.setMass(MASS);
		//		  _loadData3.setCenterOfMass(
		//		    CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
		//		    CENTER_OF_MASS_IN_MILLIMETER[2]);
		//
		//		  _toolAttachedToLBR3 = new Tool("Tool2", _loadData3);
		//
		//		  XyzAbcTransformation trans3 = XyzAbcTransformation.ofRad(TRANSLATION_OF_TOOL_3[0], TRANSLATION_OF_TOOL_3[1], TRANSLATION_OF_TOOL_3[2],TRANSLATION_OF_TOOL_2[3],TRANSLATION_OF_TOOL_2[4],TRANSLATION_OF_TOOL_2[5]);
		//		  ObjectFrame aTransformation3 = _toolAttachedToLBR3.addChildFrame(TOOL_3_FRAME + "(TCP)", trans3);
		//		  _toolAttachedToLBR3.setDefaultMotionFrame(aTransformation3);
		//		  // Attach tool to the robot
		//		  _toolAttachedToLBR3.attachTo(lbr.getFlange());
		//
		//
		//		  Frame cmdPos = lbr.getCurrentCartesianPosition(_toolAttachedToLBR3.getDefaultMotionFrame());
		//
		//		  System.out.println("1:"+cmdPos);

		
		final CartesianImpedanceControlMode cartImp = createCartImp();	
		
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
			// TODO è‡ªåŠ¨ç”Ÿæˆ�çš„ catch å�—
			e.printStackTrace();
		} catch (ExecutionException e) {
			// TODO è‡ªåŠ¨ç”Ÿæˆ�çš„ catch å�—
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
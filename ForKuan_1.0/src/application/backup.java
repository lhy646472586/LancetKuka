package application;



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
public class backup extends RoboticsAPIApplication {
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
//	@Named("gripper")
//	@Inject
//	private Tool gripper;
	
//	@Named("Tool_2")
//	@Inject
//	private Tool Tool_2;
	
	@Named("Tool_2")
	@Inject
	Tool needle;
	
//	@Named("Tool_2")
//	@Inject
//	private Tool needle;
	
	private ObjectFrame tcp;
	private ObjectFrame tcp_2;
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
    
	//å…¨å±€X,Y,Zå�˜é‡� è¾“å…¥å�˜é‡�
	public static double nX=0;
	public static double nY=0;
	public static double nZ=0;
	public static double nA=0;
	public static double nB=0;
	public static double nC=0;
	
	
	 JointPosition currentPos_test;

	//å…¨å±€å·¥ä½œæ¨¡å¼�å�˜é‡� è¾“å…¥å�˜é‡�
	public static int nWorkingmode=0;
	@Inject
	private CopyOfTeachingByHand_2 JointImpedanceMode;
	@Override
	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		SafeDataIO = new SafeDataIOGroup(kuka_Sunrise_Cabinet_1);
		nX=0;
		nY=0;
		nZ=0;
		nA=0;
		nB=0;
		nC=0;
		io.setOutput5(false);
		jointPos=new JointPosition(   Math.toRadians(-20.3),
                Math.toRadians(-40.9),
                Math.toRadians(65.6),
                Math.toRadians(52.8),
                Math.toRadians(-49.1),
                Math.toRadians(-92.1),
                Math.toRadians(142.6));
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

        // Create a Tool by Hand this is the tool we want to move with some mass
        // properties and a TCP-Z-offset of 100.
//        _loadData = new LoadData();
//        _loadData.setMass(MASS);
//        _loadData.setCenterOfMass(
//                CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
//                CENTER_OF_MASS_IN_MILLIMETER[2]);
//        _toolAttachedToLBR = new Tool("Tool", _loadData);
//
//        XyzAbcTransformation trans = XyzAbcTransformation.ofTranslation(
//                TRANSLATION_OF_TOOL[0], TRANSLATION_OF_TOOL[1],
//                TRANSLATION_OF_TOOL[2]);
//        ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME
//                + "(TCP)", trans);
//        _toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
//        // Attach tool to the robot
//        _toolAttachedToLBR.attachTo(lbr.getFlange());

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
	}

	
public  class sendRTdata implements Callable<String> {
         

		
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


	
	public  class Monitor implements Callable<String> {
         

		
		public String call() {
			
			
			
			
			while(true){
				try{
					ThreadUtil.milliSleep(20);

				
				
					
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
					
//					//å…³èŠ‚åº¦æ•°1
					JointPosition actPos = lbr.getCurrentJointPosition();
					a1 = Math.toDegrees(actPos.get(0));
					BigDecimal bigDecimal = new BigDecimal(a1);
					a1 = bigDecimal.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
					data6=String.valueOf(a1)+",";
					
//					//å…³èŠ‚åº¦æ•°2
					a1 = Math.toDegrees(actPos.get(1));
					BigDecimal bigDecimal1 = new BigDecimal(a1);
					a1 = bigDecimal1.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
					data7=String.valueOf(a1)+",";
					
//					//å…³èŠ‚åº¦æ•°3
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
					Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));
					a1=cmdPos.getX();
					BigDecimal bigDecimal7 = new BigDecimal(a1);
					a1 = bigDecimal7.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//					data13=a1+",";
					data13=String.valueOf(a1)+",";
					
					//è½´å��æ ‡y
					a1=cmdPos.getY();
					BigDecimal bigDecimal8 = new BigDecimal(a1);
					a1 = bigDecimal8.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//					data14=a1+",";
					data14=String.valueOf(a1)+",";
					
					//è½´å��æ ‡z
					a1=cmdPos.getZ();
					BigDecimal bigDecimal9 = new BigDecimal(a1);
					a1 = bigDecimal9.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//					data15=a1+",";
					data15=String.valueOf(a1)+",";
					
					//è½´å��æ ‡rx
					a1=Math.toDegrees(cmdPos.getAlphaRad());
					BigDecimal bigDecima20 = new BigDecimal(a1);
					a1 = bigDecima20.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//					data16=a1+",";
					data16=String.valueOf(a1)+",";
					
					//è½´å��æ ‡ry
					a1=Math.toDegrees(cmdPos.getBetaRad());
					BigDecimal bigDecima21 = new BigDecimal(a1);
					a1 = bigDecima21.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//					data17=a1+",";
					data17=String.valueOf(a1)+",";
					
					//è½´å��æ ‡rz

					
					a1=Math.toDegrees(cmdPos.getGammaRad());
					BigDecimal bigDecima22 = new BigDecimal(a1);
					a1 = bigDecima22.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//					data18=a1+"$";
					data18=String.valueOf(a1)+",";
					}catch(Exception ex){
						break;
					}
		
		}
			return "end";
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
		    socket_recive.setSoTimeout(2500);
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
//				        	System.out.println("oo");
				        	break;
				        }
//				        System.out.println("ee");
				        while (( size = in.read(buf,0,buf.length)) != -1) {
//				            System.out.println("ww");
				            String str = new String(buf);

				            sb.append(str);
				            System.out.print(str);
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
							System.out.println("para: " + units[2]);
							System.out.println("para: " + units[3]);
							System.out.println("para: " + units[4]);
							System.out.println("para: " + units[5]);
							System.out.println("para: " + units[6]);
							String para1 = units[7].substring(0, units[7].length() - 1);
							System.out.println("para" + para1);

							nX=Double.parseDouble(units[2]);
							nY=Double.parseDouble(units[3]);
							nZ=Double.parseDouble(units[4]);
							nA=Double.parseDouble(units[5]);
							nB=Double.parseDouble(units[6]);
							nC=Double.parseDouble(para1);
							System.out.println("nX"+nX+"  nY"+nY+"  nZ"+nZ+"  nA"+nA+"  nB"+nB+"  nC"+nC);
							writer_recive.write("$para,mp,0$");
							writer_recive.flush();
						}
						else if(units[1].equals("sIO")){
							System.out.println("para: " + units[2]);
							String para2 = units[3].substring(0, units[3].length() - 1);
							if(Double.parseDouble(para2)==1 && Double.parseDouble(units[2])==1)
							{
							io.setOutput5(true);	
//							System.out.println("io.setOutput5(true)");
							}
							else{
							io.setOutput5(false);
//							System.out.println("io.setOutput5(false)");
//							System.out.println("*"+Double.parseDouble(para2)+"*  "+"*"+Double.parseDouble(units[2])+"*");
							}
//							System.out.println("sIO: " + para2);
							writer_recive.write("$res,sIO,0$");
							writer_recive.flush();
							
						}
						else if(units[1].equals("setwm")){
							String para3 = units[2].substring(0, units[2].length() - 1);
//							System.out.println("setwm: " + para3);
							nWorkingmode=Integer.parseInt(para3);
							writer_recive.write("$res,setwm"+nWorkingmode);
							System.out.println("setwm:"+nWorkingmode);
							writer_recive.flush();
							
						}
						else if(units[1].equals("qtcp")){
							String para4 = units[2].substring(0, units[2].length() - 1);
							System.out.println("qtcp: " + para4);
							writer_recive.write("$res,qtcp,1");
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
				
//				System.out.println("closed...");					
//				System.out.println("Socket...");

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
				
				System.out.println("Socket//...");
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
		
		HandGuidingMotion motion = new HandGuidingMotion();
		motion.setJointVelocityLimit(1.2)
		.setCartVelocityLimit(1000.0).setJointLimitViolationFreezesAll(false)
		.setJointLimitsMax(Math.toRadians(10), Math.toRadians(0), Math.toRadians(45), Math.toRadians(120), +0.087,+1.571, +0.087)
		.setJointLimitsMin(Math.toRadians(-10), Math.toRadians(-60), Math.toRadians(-45), Math.toRadians(0), -0.087,-1.571, -0.087)
		.setJointLimitsEnabled(true,true,true,true,false,false,false)


		

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
		
		
	    public void moveToInitialPosition()
	    {

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

		        
		        final boolean doDebugPrints = false;

		        final JointPosition initialPosition = new JointPosition(
		                lbr.getCurrentJointPosition());

		        final SmartServo aSmartServoMotion = new SmartServo(initialPosition);

		        // Set the motion properties to 10% of the systems abilities
		        aSmartServoMotion.setJointAccelerationRel(0.1);
		        aSmartServoMotion.setJointVelocityRel(0.1);

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

		        // For Roundtrip time measurement...
		        final StatisticTimer timing = new StatisticTimer();
		        try
		        {
		            // do a cyclic loop
		            // Refer to some timing...
		            // in nanosec
		            final double omega = FREQENCY * 2 * Math.PI * 1e-9;
		            final long startTimeStamp = System.nanoTime();
		            

		            while(nWorkingmode==3)
		            {
		            	
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
		                final JointPosition currentPos = lbr.getCurrentJointPosition();
		                
		   
		                if(Math.toDegrees(currentPos.get(JointEnum.J4)) < -100){
		                	 if (controlMode instanceof JointImpedanceControlMode)
		                       {  		 
		                       final JointImpedanceControlMode jointImp = (JointImpedanceControlMode) controlMode;
		                       double para = 0;
		                       para = 150.0 / Math.toDegrees(Math.abs(Math.abs(Math.toRadians(120)) - Math.abs(currentPos.get(JointEnum.J4))));
//		                       System.out.println("xishu "+Math.toDegrees(Math.abs(Math.abs(Math.toRadians(120)) - Math.abs(currentPos.get(JointEnum.J4)))));
		                       jointImp.setStiffness(1, 1, 1, para, 1, 1, 1);
//		                       System.out.println(para);
		                       jointImp.setDamping(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
		                       theSmartServoRuntime
		                               .changeControlModeSettings(jointImp);
		                		 
		                		 
		                       }
		                }
		                else{
		                
		                    
		                	if (controlMode instanceof JointImpedanceControlMode)
		               	 	{
		               	 		final JointImpedanceControlMode jointImp = (JointImpedanceControlMode) controlMode;
		               	 		jointImp.setStiffness(1, 1, 1, 1, 1, 1, 1);
		               	 		jointImp.setDamping(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
		               	 		// Send the new Stiffness settings down to the
		               	 		// controller
		               	 		theSmartServoRuntime
		               	 		.changeControlModeSettings(jointImp);
		               	 	} 	
		                }
		                
		                
//		                
		                if(Math.toDegrees(currentPos.get(JointEnum.J4))<-110)
		                {
		                JointPosition currentPos1=new JointPosition(currentPos.get(JointEnum.J1), currentPos.get(JointEnum.J2), currentPos.get(JointEnum.J3), -1.9198, currentPos.get(JointEnum.J5), currentPos.get(JointEnum.J6), currentPos.get(JointEnum.J7));
		                destination.set(currentPos1);
		            	System.out.println("change.."+currentPos1);
		                }
		                else{
		                	destination.set(currentPos);
		                }
		                
		                
		                
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
		        final JointImpedanceControlMode jointImp = new JointImpedanceControlMode(1, 1, 1, 1, 1, 1, 1);
//		        jointImp.setStiffness(1, 1, 1, 1, 1, 1, 1);
		        jointImp.setDamping(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
		        return jointImp;
		    }
		    
		    protected CartesianImpedanceControlMode createCartImp()
		    {
		        final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
		        cartImp.parametrize(CartDOF.TRANSL).setStiffness(500.0);
		        cartImp.parametrize(CartDOF.ROT).setStiffness(50.0);
//		        cartImp.parametrize(CartDOF.X).setAdditionalControlForce(-4.9);
		        cartImp.setNullSpaceStiffness(100.);
		        
//		        cartImp.setMaxCartesianVelocity(5.0,5.0,5.0,0.2,0.2, 0.2);
		        // For your own safety, shrink the motion abilities to useful limits
		        cartImp.setMaxPathDeviation(150., 150., 50., 50., 50., 50.);
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
		        	Frame Ptest2 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));
		         	
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
		    
		
		public String call() {
//			int answer;
//			answer = getApplicationUI().displayModalDialog(
//			ApplicationDialogType.INFORMATION,"Moving Mode", "Manule","Handle");
			while (true)
			{ 
//				boolean btest=SafeDataIO.getInput4();
//				if (btest==true)
//				{
//					nWorkingmode=1;
//					System.out.println("SafeDataIO.getInput4"+btest);
//				}
//				System.out.println(btest);
				boolean btest=SafeDataIO.getInput4();
				if (btest==true)
				{
					nWorkingmode=1;
				}
				if (nWorkingmode==1){
					
					needle.getFrame("/tcp_3").move(createhandGuidingMotion());
					bDangerous=false;
					nWorkingmode=0;
				}
			    else if (nWorkingmode==2 ){
//					System.out.println("automode"+nWorkingmode);
//					Frame Ptest1= getApplicationData().getFrame("/P1").copyWithRedundancy();	
                   //testdata x:735  y:7.59  z:122 Aï¼š-91 Bï¼š-40 Cï¼š-178 $cmd,ml,715,7,122,-91,-40,-178$
					//$cmd,RobotMove,1$


//						Ptest1.setX(nX);
//						Ptest1.setY(nY);
//						Ptest1.setZ(nZ);
////						System.out.println("nx:"+nX+"  ny:"+nY+"  nz:"+nZ+"  a:"+nZ);
//						Ptest1.setAlphaRad(Math.toRadians(nA));
//						Ptest1.setBetaRad(Math.toRadians(nB));
//						Ptest1.setGammaRad(Math.toRadians(nC));
			    	   System.out.println("*1");
						final CartesianImpedanceControlMode cartImp = createCartImp();
//					
//						
////						ThreadUtil.milliSleep(500);
////						System.out.println("222");
//						  System.out.println("*2");
//						JointPosition jReady =lbr.getCurrentJointPosition();
//						System.out.println("*3");
//		                jReady.set(1, -0.35);
//		                jReady.set(2, -0.71);
//		                jReady.set(3, 1.14);
//		                jReady.set(4, 0.92);
//		                jReady.set(5, -0.85);
//		                jReady.set(6, -1.607);
//		                jReady.set(7, 2.48);
//		                System.out.println(jReady);
//		                System.out.println("*4");
////						lbr.move(ptp(jReady).setMode(cartImp).setBlendingCart(0).setJointVelocityRel(0.2).setBlendingRel(0).setBlendingRel(0));
//		                lbr.move(ptp(jReady));
//		                System.out.println("*5");
		                lbr.moveAsync(new PTP(jointPos).setJointVelocityRel(0.2).setMode(cartImp));
		                System.out.println("*2");
		                nWorkingmode=0;
		                System.out.println("*3");
		               
		                
//					Frame Ptest2 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy().transform((Transformation.ofTranslation(0, 20, 0)));
			    	
			    	//随动模式
//					 getLogger().info("Move to start position.");
////				        lbr.move(ptp(0., Math.PI / 180 * 30., 0., -Math.PI / 180 * 60., 0.,
////				                Math.PI / 180 * 90., 0.).setJointVelocityRel(0.1));
//				        AbstractFrame initialPosition = lbr.getCurrentCartesianPosition(lbr
//				                .getFlange());
//				        // Create a new smart servo linear motion
//				        SmartServoLIN aSmartServoLINMotion = new SmartServoLIN(initialPosition);
//
//				        aSmartServoLINMotion.setMinimumTrajectoryExecutionTime(20e-3);
//				        
//				        getLogger().info("Starting the SmartServoLIN in position control mode");
//				        lbr.getFlange().moveAsync(aSmartServoLINMotion);
//
//				        getLogger().info("Get the runtime of the SmartServoLIN motion");
//				        _smartServoLINRuntime = aSmartServoLINMotion.getRuntime();
//				        
//				        StatisticTimer timing = new StatisticTimer();
//
//				        // Start the smart servo lin sine movement
//				        timing = startSineMovement(_smartServoLINRuntime, timing);
//				  
//				        ThreadUtil.milliSleep(1000);
//
//				        getLogger().info("Print statistic timing");
//				        getLogger()
//				                .info(getClass().getName() + _smartServoLINRuntime.toString());
//
//				        getLogger().info("Stop the SmartServoLIN motion");
//				        _smartServoLINRuntime.stopMotion();
//
//				        // Statistic Timing of sine movement loop
//				        if (timing.getMeanTimeMillis() > 150)
//				        {
//				            getLogger()
//				                    .info("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
//				            getLogger()
//				                    .info("Under Windows, you should play with the registry, see the e.g. user manual");
//				        }
				
				}
				
				else if(nWorkingmode==3){
//			        // Initialize Joint impedance mode    
//					System.out.println("JointimplentMode"+nWorkingmode);
//					moveToInitialPosition();
//			        final JointImpedanceControlMode jointImp = createJointImp();
//			        runSmartServoMotion(jointImp);
					
//					Frame Ptest1= getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();		
//					needle.getFrame("/tcp_2").move(ptp(Ptest1).setBlendingCart(0).setJointVelocityRel(0.2).setBlendingRel(0).setBlendingRel(0));
//					System.out.println("automode"+nWorkingmode);
//					Frame Ptest1= getApplicationData().getFrame("/P1").copyWithRedundancy();
					
					
					Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));

                   //testdata x:735  y:7.59  z:122 Aï¼š-91 Bï¼š-40 Cï¼š-178 $cmd,ml,715,7,122,-91,-40,-178$
					//$cmd,RobotMove,1$
					if(nX!=0 && nY!=0 && nZ!=0){
						Ptest1.setX(nX);
						Ptest1.setY(nY);
						Ptest1.setZ(nZ);
//						System.out.println("nx:"+nX+"  ny:"+nY+"  nz:"+nZ+"  a:"+nZ);
						Ptest1.setAlphaRad(Math.toRadians(nA));
						Ptest1.setBetaRad(Math.toRadians(nB));
						Ptest1.setGammaRad(Math.toRadians(nC));

						
						if(Math.abs(nX)<2000 && Math.abs(nY)<2000 && Math.abs(nZ)<2000 && Math.abs(nA)<2000 && Math.abs(nB)<2000 && Math.abs(nC)<2000){
							needle.getFrame("/tcp_2").move(ptp(Ptest1).setJointVelocityRel(0.2));	
						}
						else{
							System.out.println("Err_DangerPlace: "+"nX:"+nX+"nY:"+nY+"nZ:"+nZ+"nA:"+nA+"nB:"+nB+"nC:"+nC);
						}
						
						
//						ThreadUtil.milliSleep(500);
//						System.out.println("222");
					}
					else{
						ThreadUtil.milliSleep(10);
					}
					

				}
				else if(nWorkingmode==4){
			        // Initialize Cartesian impedance mode    
					System.out.println("CartesianimplentMode"+nWorkingmode);
					moveToInitialPosition();
					final CartesianImpedanceControlMode cartImp = createCartImp();
			        runSmartCartesianMotion(cartImp);
			  if(bDangerous==true){
				  nWorkingmode=1;
				  System.out.println("WorkingModeForceToSet1");
			  }

				}
				else{
					ThreadUtil.milliSleep(500);
				}

			
			

			
			}
			//return "end get";
		}

	}

	
	
	
	
	//ä¿¡å�·é‡‡é›†
	
	
	public void MotionType() {
		// TODO è‡ªåŠ¨ç”Ÿæˆ�çš„æ–¹æ³•å­˜æ ¹
		
	}

	
	public void run() {
		
		JointPosition actPos = lbr.getCurrentJointPosition();
		
//		Vector vec=Vector.of(841.79, -84.76, 178.95);
//		Matrix translation = Matrix.ofRowFirst(0.302, -0.933, -0.194, -0.302, -0.287, 0.909, -0.904, -0.216, -0.368);
//		MatrixTransformation trans2=MatrixTransformation.of(vec, translation);
////		Transformation trans1 = Transformation.of(matrix);
////		needle.getInverseKinematic(trans2, actPos);
//	    
//		
//
//		System.out.println("start");
//		System.out.println(trans2);
//		System.out.println(actPos);
		
//		motion = handGuiding()
//				 .setJointLimitsMax(+1.407, +0.872, +0.087, -0.785, +0.087,
//				 +1.571, +0.087)
//				 .setJointLimitsMin(-1.407, +0.175, -0.087, -1.571, -0.087,
//				 -1.571, -0.087)
//				 .setJointLimitsEnabled(false, true, false, true, false,
//				 true, false)
//				 .setJointLimitViolationFreezesAll(false)
//				 .setPermanentPullOnViolationAtStart(true);
//		lbr.move(motion);
//		//lbr.move(handGuiding());
//		System.out.close();
//		JointPosition actPos = lbr.getCurrentJointPosition();
//		//tcp.move(ptp(getApplicationData().getFrame("/P2")).setJointVelocityRel(.3));
//		
//		Frame Ptest1= getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();		
//		Frame Ptest2 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy().transform((Transformation.ofTranslation(0, 200, 0)));
//		Frame Ptest3 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy().transform((Transformation.ofTranslation(-200, 200, 0)));
//		Frame Ptest4 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy().transform((Transformation.ofTranslation(-200, 0, 0)));
//		Frame Ptest5 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy().transform((Transformation.ofTranslation(0, 0, 200)));
//		Frame Ptest6 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy().transform((Transformation.ofTranslation(0, 200, 200)));
//		Frame Ptest7 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy().transform((Transformation.ofTranslation(-200, 200, 200)));
//		Frame Ptest8 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy().transform((Transformation.ofTranslation(-200, 0, 200)));
//		
//		//ThreadUtil.milliSleep(5000);//frequency of recording
////		needle.getFrame("/tcp_2").move(ptp(Ptest1).setBlendingCart(0).setJointVelocityRel(0.2).setBlendingRel(0).setBlendingRel(0));
//		currentPos_test=lbr.getCurrentJointPosition();

		ExecutorService executor = Executors.newCachedThreadPool();
		Future<String> add = executor.submit(new sendRTdata());
		Future<String> say = executor.submit(new motion());
		Future<String> sdd2 = executor.submit(new reciveRTdata());
		Future<String> sdd3 = executor.submit(new Monitor());

		try {
			System.out.println(add.get());
			System.out.println(say.get());
			System.out.println(sdd2.get());
			System.out.println(sdd3.get());
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
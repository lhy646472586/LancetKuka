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

import java.net.DatagramPacket;
import java.net.InetAddress;
import java.net.MulticastSocket;
import java.util.Date;

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
public class testForxi_2 extends RoboticsAPIApplication {
//    int i=0;
	private HandGuidingMotion motion;


	@Inject
	private  LBR lbr;
	
	@Override
	public void initialize() {
		
        

	}

	 /**
     * multicastClient方法描述:组播客户端实现，用于向加入的组播其它用户发送组播消息
     * 
     * @author : Ricky
     * @createTime : Apr 13, 2016 8:44:58 AM
     * @throws Exception
     */
    public static void multicastClient() throws Exception {
        InetAddress group = InetAddress.getByName("172.31.1.147");// 组播地址
        int port = 30005;// 组播端口
        MulticastSocket mss = null;// 创建组播套接字
        try {
            mss = new MulticastSocket(port);
            mss.joinGroup(group);
            System.out.println("发送数据包启动！（启动时间" + new Date() + ")");
 
            String message = "Box_ " + new Date();
            byte[] buffer = message.getBytes();
            DatagramPacket dp = new DatagramPacket(buffer, buffer.length,
                    group, port);
            mss.send(dp);
            System.out.println("发送数据包给 " + group + ":" + port);
            Thread.sleep(5000);
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            try {
                if (mss != null) {
                    mss.leaveGroup(group);
                    mss.close();
                }
            } catch (Exception e2) {
                e2.printStackTrace();
            }
        }
    }
 
    /**
     * multicastServer方法描述:组播服务端实现，用于监听组播中的消息，在接收到消息以后再向组播中的其它成员反馈消息
     * 
     * @author : Ricky
     * @createTime : Apr 13, 2016 8:45:25 AM
     * @throws Exception
     */
    public static void multicastServer() throws Exception {
        InetAddress group = InetAddress.getByName("172.31.1.147");// 组播地址
        int port = 30005;// 组播端口
        MulticastSocket msr = null;// 创建组播套接字
        try {
            msr = new MulticastSocket(port);
            msr.joinGroup(group);// 加入连接
            byte[] buffer = new byte[8192];
            System.out.println("接收数据包启动！(启动时间: " + new Date() + ")");
            while (true) {
                // 建立一个指定缓冲区大小的数据包
                DatagramPacket dp = new DatagramPacket(buffer, buffer.length);
                msr.receive(dp);
                String s = new String(dp.getData(), 0, dp.getLength());
                // 解码组播数据包
                System.out.println("接收到的组播数据包是：" + s);
                multicastClient();
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            if (msr != null) {
                try {
                    msr.leaveGroup(group);
                    msr.close();
                } catch (Exception e2) {
                    e2.printStackTrace();
                }
            }
        }
    }


	
	public void run() {
		try {
            multicastServer();
        } catch (Exception e) {
            e.printStackTrace();
        }

	}
}
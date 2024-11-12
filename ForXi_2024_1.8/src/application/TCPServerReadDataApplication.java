package application;


import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPITask;
import com.kuka.roboticsAPI.applicationModel.tasks.UseRoboticsAPIContext;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a {@link RoboticsAPITask#run()} method, which will be called
 * successively in the application lifecycle. The application will terminate automatically after the {@link RoboticsAPITask#run()} method
 * has finished or after stopping the task. The {@link RoboticsAPITask#dispose()} method will be called, even if an exception is thrown
 * during initialization or run.
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the {@link RoboticsAPITask#dispose()} method.</b>
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class TCPServerReadDataApplication extends RoboticsAPIApplication {

	private ServerSocket serverSocket;
	private Socket socket;
	@Override
	public void initialize() 
	
	{try {
		serverSocket = new ServerSocket(30003);
	} catch (IOException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
	 try {
		socket = serverSocket.accept();
	} catch (IOException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
	}

	@Override
	public void run() 
	{
		getLogger().info("Starting Application.");
		
		try {
			
			
			getLogger().info("Socket accepted. IP:{" + socket.getInetAddress().getHostAddress() + "}.");
			BufferedReader reader = new BufferedReader(new InputStreamReader(socket.getInputStream()));
			
			for (int i = 0; i < 4; i++) 
			{
				String data = reader.readLine();
				// fuehrende/nachgelagerte leerzeichen und steuerzeichen entfernen
				data = data.trim();
				// Konvertierung von String zu Double
				double value = Double.valueOf(data);
				getLogger().info("Recieved data:{" + value + "}");
			}
			
			reader.close();
			getLogger().info("Closing socket.");
			socket.close();

		} catch (IOException e) {
			getLogger().error("Fehler", e);
		}
		
		getLogger().info("Ending Application.");
	}

	@Override
	public void dispose() {
		// TODO Auto-generated method stub
		super.dispose();
		try {
			socket.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}

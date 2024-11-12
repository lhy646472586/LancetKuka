package application;


import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.Socket;

public class TCPClientRead 
{
	
	public static void main(String[] args) 
	{
		System.out.println("Starting Client.");
		try {
			Socket clientSocket = new Socket("172.31.1.147", 30001);
			System.out.println("Connection established.");
			
			BufferedReader reader = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
			while (!clientSocket.isClosed())
			{
				String data = reader.readLine();
				System.out.println("Recived from Server: {" + data + "}.");
				if(data == null ){
					clientSocket.close();
				}
			}
			
			reader.close();
			clientSocket.close();
			System.out.println("Socket closed.");
			
		} catch (IOException e){
			e.printStackTrace();
		}
		
		System.out.println("Ending Client.");
	}
	
}

package application;


import java.io.DataOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.net.Socket;

public class TCPClientSend 
{
	
	public static void main(String[] args) 
	{
		System.out.println("Starting Client.");
		try {
			Socket clientSocket = new Socket("172.31.1.147", 30001);
			System.out.println("Connection established.");
			
			DataOutputStream outputStream = new DataOutputStream(clientSocket.getOutputStream());
			OutputStreamWriter writer = new OutputStreamWriter(outputStream);
			for(int i=0; i<4; i++){
				String data = (i * Math.PI) + "\r\n";
				System.out.println("Recived from Server: {" + data + "}.");
				writer.write(data);
				writer.flush();
			}
			writer.close();
			clientSocket.close();
			System.out.println("Socket closed.");
			
		} catch (IOException e){
			e.printStackTrace();
		}
		
		System.out.println("Ending Client.");
	}
	
}

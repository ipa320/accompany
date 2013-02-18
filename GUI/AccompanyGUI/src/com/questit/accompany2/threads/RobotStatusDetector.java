package com.questit.accompany2.threads;


import java.io.IOException;
import java.io.InputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;

import com.questit.accompany2.AccompanyGUI_app2;

import android.os.Handler;
import android.util.Log;

public class RobotStatusDetector extends Thread {

	protected AccompanyGUI_app2 myApp;
	//protected String ip;
	protected int port;
	
	ServerSocket mySS;
	
	protected Handler handler;
	
	protected boolean stop;
	
	public RobotStatusDetector(String IP, int Port,AccompanyGUI_app2 a,Handler h)
	{
		this.myApp=a;
		//this.ip=IP;
		this.port=Port;
		this.stop=false;
		Log.i("AccompanyGUI","new RSD!");
		this.handler=h;
	}
	
	@Override
	public void run()
	{
		try {
		 mySS = new ServerSocket(port);
			Log.e("AccompanyGUI","RSD listening on: "+ port);
			while(!stop)
        	{
				Log.i("AccompanyGUI","RSD: waiting for client connection... (RSD stop: "+stop+")");
				Socket client = mySS.accept();
				Log.i("AccompanyGUI","RSD: connected from "+client.getInetAddress().getHostAddress()+"...");
				
				InputStream i=null;
				try {
					i = client.getInputStream();
				} catch (IOException e1) {
					Log.e("AccompanyGUI","ERROR: cannot get the client socket buffer");
				}
				int bytesread=0;
				
				Log.i("AccompanyGUI","RSD: try to read from client...");
				while (bytesread<1)
				{
					Log.i("A_GUI","RSD: reading....");
					bytesread=i.read();
				}
				Log.e("AccompanyGUI","ack received");
				client.close();
				
				handler.postAtFrontOfQueue( new Runnable(){
					@Override
					public void run() {
						myApp.robotFree();
					}
				});
				Log.i("AccompanyGUI","RSD: end of cylce, stop: "+stop);
			
        	}
		} catch (UnknownHostException e) {
			Log.e("connection","U.H.");
		} catch (IOException e) {
			Log.e("connection","I.O.");
		}
        
	}
	
	public void halt()
	{
		this.stop=true;
		if (mySS!= null)
			try {
				mySS.close();
			} catch (IOException e) {
				Log.e("AccompanyGUI","closing server socket exception!");
			}
		this.interrupt();
	}
	
	public int getRsdPort()
	{
		return port;
	}
	
}
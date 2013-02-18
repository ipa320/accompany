package com.questit.accompany2.gesture_detection;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;

import com.questit.accompany2.activities.RobotView;

import android.util.Log;
import android.view.MotionEvent;
import android.view.GestureDetector.SimpleOnGestureListener;

public class myGestureDetector extends SimpleOnGestureListener {
	
	protected final int SWIPE_MIN_DISTANCE=100;
	protected final int SWIPE_THRESHOLD_VELOCITY=200;
	
	protected String robot_ip;
	protected int robot_port;
	protected RobotView act;
	
	public myGestureDetector(String ip, int port,RobotView ac)
	{
		super();
		robot_ip=ip;
		robot_port=port;
		act=ac;
	}
	
    @Override
    public boolean onFling(MotionEvent e1, MotionEvent e2, float velocityX,
            float velocityY) {
        
    	try {
        	
        	boolean dirty=false;
        	String cmd=null;
        
            // right to left swipe
            if (e1.getX() - e2.getX() > SWIPE_MIN_DISTANCE
                    && Math.abs(velocityX) > SWIPE_THRESHOLD_VELOCITY) {
               Log.e("Gesture","detected 1 "+(e1.getX() - e2.getX()));
               dirty=true;  
               //cmd="turn "+(e1.getX() - e2.getX());
               cmd="turn "+(e2.getX() - e1.getX());
            } else if (e2.getX() - e1.getX() > SWIPE_MIN_DISTANCE
                    && Math.abs(velocityX) > SWIPE_THRESHOLD_VELOCITY) {
            			Log.e("Gesture","detected 2 "+(e2.getX() - e1.getX())); 
            			dirty=true;
            			//cmd="turn "+(e1.getX() - e2.getX());
            			cmd="turn "+(e2.getX() - e1.getX());
            		}
            		else{
            			//do nothing: too little fling. 
            		}
            
            //up-down swipe
            if ((Math.abs(e1.getY() - e2.getY()) > SWIPE_MIN_DISTANCE)&&
            		Math.abs(velocityY)>SWIPE_THRESHOLD_VELOCITY)
            {
            	if (cmd!=null) cmd=cmd+" ";
            	else cmd="";
            	Log.e("Gesture","detected Y "+(e1.getY() - e2.getY()));
            	//double movement=act.getCameraPosition()+(double) ((e1.getY() - e2.getY())/750.0);
            	double movement=act.getCameraPosition()+(double) ((e2.getY() - e1.getY())/750.0);
            	double torso=0.;
            	if (movement>0) 
            	{
            		torso=act.getTorsoPosition()+(movement)*0.2;//poi get
            		if (torso>1) torso=1;
            		movement=0.;
            	}
            	if (movement<(-3.1415926))
            	{
            		torso=act.getTorsoPosition()+(movement+3.1415926)*0.2;
            		if (torso<-1) torso=-1;
            		
            		movement=-3.1415926;
            	}
            	cmd=cmd+"camera "+movement+" "+torso;
            	act.setCameraPosition(movement);
            	act.setTorsoPosition(torso);
            	dirty=true;
            }
            
            if (dirty)
            {
            	try{
            		
             	   Log.i("INFO","socket da creare ");
    				   Socket COB_sock=new Socket(robot_ip,robot_port);
    				   //COB_sock=new Socket("192.168.1.104",CareOBotPort);
    				   Log.i("INFO","socket creata");
    				   //sending the command to the robot (remote command server)
    				   if (COB_sock!=null)
    				   {
    					   Log.e("connnnn","clicked");
    					   Log.i("conn","connected" +
    							   "");
    					   PrintWriter os = new PrintWriter(COB_sock.getOutputStream());
    					   //os.write("turn " + (e1.getX() - e2.getX()));
    					   os.write(cmd);
    					   os.flush();
    					   os.close();
    					   COB_sock.close();
    					   COB_sock=null;
    				   }
    				   act.showRobotExecutingCommandView("turn");
                } catch (UnknownHostException e) {
             	   Log.e("ERR","Unknown host for the Care-o-bot");
             	   //toastMessage("Please set the correct Ip and port for the care-o.bot");
    				} catch (IOException e) {
    					Log.e("ERR","IO Exception for the Care-o-bot");
    				}
            	dirty=false;
            }
            
        } catch (Exception e) {
            Log.e("Gesture","problem");
        }
        return false;
    }
}

package com.questit.accompany2;

import java.net.URI;

import org.ros.address.InetAddressFactory;
import org.ros.android.AccompanyBitmapFromCompressedImage;
import org.ros.android.AccompanyRosApp;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.NodeMainExecutorService;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;

import com.questit.accompany2.RosNodes.AccompanyActionsClient;
import com.questit.accompany2.RosNodes.CmdVelocityPublisher;
import com.questit.accompany2.RosNodes.DatabaseClient;
import com.questit.accompany2.RosNodes.HeadControllerGUI;
import com.questit.accompany2.RosNodes.ImagesSubscriber;
import com.questit.accompany2.RosNodes.TorsoControllerGUI;
import com.questit.accompany2.activities.ActionsListView;
import com.questit.accompany2.activities.LoginPage;
import com.questit.accompany2.activities.RobotView;
import com.questit.accompany2.activities.RobotWorkingView;
import com.questit.accompany2.activities.UserView;

import android.app.Application;
import android.app.ProgressDialog;
import android.content.Context;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.os.Handler;
import android.util.Log;
import android.view.Display;
import android.view.WindowManager;

public class AccompanyGUI_app2 extends AccompanyRosApp{

	//Activities identifiers
	public final int USER_VIEW=1;
	public final int ROBOT_VIEW=2;
	public final int ACTIONS_VIEW=3;
	public final int EXECUTE_VIEW=4;
	
	//DB reqeusts identifiers
	public final int ALL_ACTIONS_REQUEST_CODE=1;
	public final int USER_ACTIONS_REQUEST_CODE=2;
	public final int ROBOT_ACTIONS_REQUEST_CODE=3;
	public final int SONS_REQUEST_CODE=4;
	
	//The user id saved after login (don't touch it!!)
	protected int user_id;
	
	//the running activity
	protected int running=0;
	protected int old_running;
	
	//the app's activities
	protected UserView  user_act  = null;
	protected RobotView robot_act = null;
	protected RobotWorkingView robot_working_view= null;
	protected ActionsListView actions_list= null;
	protected LoginPage login_page = null;
	//app's Thread
	
	protected Handler rsd_handler;
	
	//current Bitmap from robot
	protected Bitmap robot_image;
	protected Handler images_handler;
	
	public CmdVelocityPublisher turn_talker;
	public HeadControllerGUI head_controller;
	public TorsoControllerGUI torso_controller;
	protected ImagesSubscriber<sensor_msgs.CompressedImage> imagesSubscriber;
	
	public AccompanyActionsClient aac;
	protected Handler actions_handler;
	public DatabaseClient db_client;
	protected Handler db_handler; 
	
	//screen dimesions:
	int dw;
	int dh;
	
	protected int on_last_son_req_activity_running;
	protected String RosMasterIP;
	protected int rate;
	
	//app's preferences ()
	
	@Override
	public void onCreate()
	{
		super.onCreate();
		
	}
	
	public void setRunningActivity(int r)
	{
		running=r;
	}
	
	public void setLoginPage(LoginPage lp)
	{
		AccompanyPreferences pref= new AccompanyPreferences(lp);
		pref.loadPreferences();
		rate=pref.getImagesRate();
		this.login_page=lp;
		dh= login_page.getDisplayHeight();
		dw= login_page.getDisplayWidth();
		//if the app is starting the first time, calls the NodeMainExecutor creation
	}
	
	public void startServices(String ip,int r)
	{
		rate=r;
		RosMasterIP=ip;
		//setURI(RosMasterIP);
		setURI2(RosMasterIP);
		Log.e("URI",getURI());
		images_handler= new Handler();
		db_handler= new Handler();
		actions_handler= new Handler();		
		startNodeMainExecutorService();
	}
	
	public void setUserView(UserView u)
	{
		/*startNodeMainExecutorService();
		this.user_act=u;
		dh= user_act.getDisplayHeight();
		dw= user_act.getDisplayWidth();
		//if the app is starting the first time, calls the NodeMainExecutor creation
		images_handler= new Handler();
		db_handler= new Handler();
		actions_handler= new Handler();
		//and also the rsd:
		if (rsd==null)
		{
			rsd_handler=new Handler();
			rsd= new RobotStatusDetector(user_act.getPreferences().getRobotIP(),
					user_act.getPreferences().getRSDPort(),this,rsd_handler);
			rsd.start();
		}*/
		this.user_act=u;
	}
	
	public void robotBusy()
	{
		old_running=running;
		running= EXECUTE_VIEW;
	}
	
	public void setActionsView(ActionsListView a)
	{
		actions_list=a;
	}
	
	public void robotFree()
	{
		Log.e("AccompanyGUI","switching back to "+old_running);
		if (running!=EXECUTE_VIEW) Log.e("AccompanyGUI","ack already received");
		else
			switch (old_running)
			{
				case ROBOT_VIEW:
				{
					robot_working_view.switchToRobotView();
				} break;
				case USER_VIEW:
				{
					robot_working_view.switchToUserView();
				} break;
				case ACTIONS_VIEW:
				{
					robot_working_view.switchToActionsList();
				} break;
				case EXECUTE_VIEW:
				{
					Log.e("AccompanyGUI","GRAVE: old_running must never be equal to EXECUTE_VIEW!!!");
				}
			}
	}
	
	public void setRobotView(RobotView r)
	{
		this.robot_act=r;
	}
	
	public void setRobotWorkingView(RobotWorkingView r)
	{
		this.robot_working_view=r;
	}
	
	public int getDisplayWidth()
	{
		Display display = ((WindowManager) getSystemService(Context.WINDOW_SERVICE)).getDefaultDisplay();
	    return display.getWidth(); 
	}
	
	public int getDisplayHeight()
	{
		Display display = ((WindowManager) getSystemService(Context.WINDOW_SERVICE)).getDefaultDisplay();
	    return display.getHeight(); 
	}
	
	public void setBitmap(Bitmap b)
	{
		if (b!=null)
		{
			if (robot_image!=null) robot_image.recycle();
			/*float scaleW= (float)dw / b.getWidth();
			float scaleH= (float)dh / b.getHeight();*/
			//Log.i("Accompany-RobotImage",dw+" "+dh);
			//Log.i("Accompany-RobotImage",b.getWidth()+" "+b.getHeight());
			//Log.i("Accompany-RobotImage",scaleW+","+scaleH);
			/*Matrix mm= new Matrix();*/
			//mm.postScale(scaleW, scaleH);
			/*if (head_controller.getHeadPos() > -(head_controller.max_head_pos-head_controller.min_head_pos)/2)
				mm.setRotate(180);*/
			//robot_image=Bitmap.createBitmap(b,0,0,dw,dh,mm,false);
			//robot_image=Bitmap.createBitmap(b,0,0,b.getWidth(),(int)(b.getHeight()*scaleH),mm,false);
			robot_image=b;
			/*b.recycle();
			b=null;
			System.gc();*/
			if (running==this.ROBOT_VIEW)
			{
				robot_act.refreshImage(robot_image);
			}
			if (running==this.EXECUTE_VIEW)
			{
				if (robot_working_view==null)
					Log.e("AccompanyGUI","ahi ahi");
				else
					robot_working_view.refreshImage(robot_image);
			}
		}
	}
	
	
	public void setBitmap2(Bitmap b,Matrix m)
	{
		if (b!=null)
		{
			if (robot_image!=null) robot_image.recycle();
			/*float scaleW= (float)dw / b.getWidth();
			float scaleH= (float)dh / b.getHeight();*/
			//Log.i("Accompany-RobotImage",dw+" "+dh);
			//Log.i("Accompany-RobotImage",b.getWidth()+" "+b.getHeight());
			//Log.i("Accompany-RobotImage",scaleW+","+scaleH);
			/*Matrix mm= new Matrix();*/
			//mm.postScale(scaleW, scaleH);
			/*if (head_controller.getHeadPos() > -(head_controller.max_head_pos-head_controller.min_head_pos)/2)
				mm.setRotate(180);*/
			//robot_image=Bitmap.createBitmap(b,0,0,dw,dh,mm,false);
			//robot_image=Bitmap.createBitmap(b,0,0,b.getWidth(),(int)(b.getHeight()*scaleH),mm,false);
			robot_image=b;
			/*b.recycle();
			b=null;
			System.gc();*/
			if (running==this.ROBOT_VIEW)
			{
				robot_act.refreshImage2(robot_image,m);
			}
			if (running==this.EXECUTE_VIEW)
			{
				if (robot_working_view==null)
					Log.e("AccompanyGUI","ahi ahi");
				else
					robot_working_view.refreshImage2(robot_image,m);
			}
		}
	}
	
	
	public Bitmap getLastImage()
	{
		return robot_image;
	}
	
	/*public void restartThreads(AccompanyPreferences newPrefs)
	{
		if (rsd.getRsdPort()!=newPrefs.getRSDPort())
		{
			rsd.halt();
			rsd= new RobotStatusDetector(newPrefs.getRobotIP(),
					newPrefs.getRSDPort(),this,rsd_handler);
			rsd.start();
		}
	}*/
	
	public void RequestToDB(int i, int son_id)
	{
		if (i==ALL_ACTIONS_REQUEST_CODE) db_client.request(i,"","-1");
		else if (i==USER_ACTIONS_REQUEST_CODE) db_client.request(i, db_client.getUserLocation(),"-1");
			 else if (i == ROBOT_ACTIONS_REQUEST_CODE) db_client.request(i, db_client.getRobotLocation(),"-1");
			 	else if (i==SONS_REQUEST_CODE)
			 	{
			 		on_last_son_req_activity_running=running;
			 		db_client.request(i, Integer.toString(son_id), Integer.toString(son_id));
			 	}
	}
	
	public void handleResponse(String response,int code,int son)
	{
		Log.i("app-response","res: "+response+", code: "+code+", "+"running: "+running);
		if ((running==ACTIONS_VIEW)&&(code==ALL_ACTIONS_REQUEST_CODE))
			actions_list.handleResponse(response);
		if ((running==USER_VIEW)&&(code==USER_ACTIONS_REQUEST_CODE))
			user_act.handleResponse(response);
		if ((running==ROBOT_VIEW)&&(code==ROBOT_ACTIONS_REQUEST_CODE))
			robot_act.handleResponse(response);
		if (code==SONS_REQUEST_CODE)
		{
			if ((running==ACTIONS_VIEW)&&(on_last_son_req_activity_running==running))
				actions_list.handleSonResponse(response,son);
			if ((running==USER_VIEW)&&(on_last_son_req_activity_running==running))
				user_act.handleSonResponse(response,son);
			if ((running==ROBOT_VIEW)&&(on_last_son_req_activity_running==running))
				robot_act.handleSonResponse(response,son);
		}
	}
	
	//the method to close the application (i.e. all the activities)
	@Override
	public void closeApp()
	{
		if (user_act  != null) user_act.halt();
		if (robot_act != null) robot_act.halt();
		if (robot_working_view != null) robot_working_view.halt();
		if (actions_list != null) actions_list.halt();
		if (login_page!= null) login_page.finish();

		closeNodeMainExecutorService();
		if (robot_image!=null) robot_image.recycle();
		robot_image=null;
		System.gc();
		android.os.Process.killProcess(android.os.Process.myPid());
	}

	@Override
	public void init(NodeMainExecutorService nmes) {
		try{
			//dw=this.getDisplayWidth();
			//dh=this.getDisplayHeight();
			dw=1920;
			dh=1200;
			aac= new AccompanyActionsClient(this,actions_handler);
			db_client= new DatabaseClient(this,db_handler);
			turn_talker= new CmdVelocityPublisher();
			head_controller = new HeadControllerGUI();
			torso_controller = new TorsoControllerGUI();
			imagesSubscriber= new ImagesSubscriber<sensor_msgs.CompressedImage>(images_handler,this,rate);
			imagesSubscriber.setTopicName("/stereo/left/image_color/compressed");
			//imagesSubscriber.setTopicName("cam3d/rgb/image_color/compressed");
		    imagesSubscriber.setMessageType(sensor_msgs.CompressedImage._TYPE);
		    imagesSubscriber.setMessageToBitmapCallable(new AccompanyBitmapFromCompressedImage(dw,dh));
			//NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),
			//	    new URI("http://"+RosMasterIP+":11311/"));
		    Log.i("INFO gui","ip "+RosMasterIP);
		    Log.i("INFO gui","URI "+new URI(RosMasterIP));
		    NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),
		    		//new URI(RosMasterIP));
		    		new URI("http://"+RosMasterIP+":11311/"));
			nmes.execute(imagesSubscriber, nodeConfiguration.setNodeName("RobotView_image"));
			nmes.execute(turn_talker, nodeConfiguration.setNodeName("CmdVelPub"));
			nmes.execute(head_controller, nodeConfiguration.setNodeName("HeadControllerGUI"));
			nmes.execute(torso_controller, nodeConfiguration.setNodeName("TorsoControllerGUI"));
			nmes.execute(aac, nodeConfiguration.setNodeName("ActionsClient"));
			nmes.execute(db_client, nodeConfiguration.setNodeName("DBclient"));
			
			Log.e("AccompanyGUI","ROS services started");
			
		}catch(Exception e)
		{
			Log.e("AccompanyGUI application","GRAVE: error starting ROS subscribers and publishers!!");
		}
	}
	
	public void sendActionRequest(String command,String phrase)
	{
		showRobotExecutingCommandView(phrase);
		aac.sendRequest(command);
	    //go back
	}
	
	protected void showRobotExecutingCommandView(String phrase)
	{
		switch (running)
		{
			case ROBOT_VIEW:
			{
				robot_act.showRobotExecutingCommandView(phrase);
			} break;
			case USER_VIEW:
			{
				user_act.showRobotExecutingCommandView(phrase);
			} break;
			case ACTIONS_VIEW:
			{
				actions_list.showRobotExecutingCommandView(phrase);
			} break;
			case EXECUTE_VIEW:
			{
				Log.e("AccompanyGUI","GRAVE: running must never be equal to EXECUTE_VIEW when you are sendinga command!!!");
			}
		}
	}
	
	public void handleActionResponse()
	{
		robotFree();
	}
	
	public void handleFailedActionResponse()
	{
		Log.e("AccompanyGUI","switching back to "+old_running);
		if (running!=EXECUTE_VIEW) Log.e("AccompanyGUI","ack already received");
		else
			switch (old_running)
			{
				case ROBOT_VIEW:
				{
					robot_working_view.switchToRobotView();
					robot_act.toastMessage("command failed!");
				} break;
				case USER_VIEW:
				{
					robot_working_view.switchToUserView();
					user_act.toastMessage("command failed!");
				} break;
				case ACTIONS_VIEW:
				{
					robot_working_view.switchToActionsList();
					actions_list.toastMessage("command failed!");
				} break;
				case EXECUTE_VIEW:
				{
					Log.e("AccompanyGUI","GRAVE: old_running must never be equal to EXECUTE_VIEW!!!");
				}
			}
	}
	
	public void setUserId(int uid)
	{
		user_id=uid;
	}
	
	public int getUid()
	{
		return user_id;
	}
	
	public void handleLoginResponse(String a,int i1,int i2)
	{
		login_page.loginResult(a);
	}
	
	public boolean getImageRotation()
	{
		if (head_controller.getHeadPos() > -(head_controller.max_head_pos-head_controller.min_head_pos)/2)
			return true;
		else return false;
		
	}
	
	public void StartSubscribing()
	{
		imagesSubscriber.startSubscribing();
	}
	
	public void stopSubscribing()
	{
		imagesSubscriber.stopSubscribing();
	}
	
	public void restartServices(AccompanyPreferences preferences)
	{
		RosMasterIP=preferences.getRosMasterIP();
		rate=preferences.getImagesRate();
		setFlagSettings();
		/*android.os.Process.killProcess(imagesSubscriber.getMyPid());
		android.os.Process.killProcess(aac.getMyPid());
		android.os.Process.killProcess(turn_talker.getMyPid());
		android.os.Process.killProcess(head_controller.getMyPid());
		android.os.Process.killProcess(torso_controller.getMyPid());*/
		/*closeRosNode(imagesSubscriber);
		closeRosNode(aac); 
		closeRosNode(db_client);
		closeRosNode(turn_talker);
		closeRosNode(head_controller);
		closeRosNode(torso_controller);*/
		closeNodeMainExecutorService();
		startNodeMainExecutorService();
	}
	
	public void setImagesRate(int ir)
	{
		imagesSubscriber.setRate(ir);
	}
	
	public void closeAppOnError()
	{
		login_page.closeAppOnError();
	}
	
}

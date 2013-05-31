package it.unisi.accompany;

import it.unisi.accompany.activities.ActionsListView;
import it.unisi.accompany.activities.LoginPage;
import it.unisi.accompany.activities.RobotView;
import it.unisi.accompany.activities.RobotWorkingView;
import it.unisi.accompany.activities.UserView;
import it.unisi.accompany.clients.DatabaseClient;
import it.unisi.accompany.rosnodes.*;
import it.unisi.accompany.threads.ActionPossibilitiesUpdateThread;
import it.unisi.accompany.threads.MaskExpressionThread;
import it.unisi.accompany.threads.RobotStatusThread;
import it.unisi.accompany.threads.SpeechThread;

import java.io.IOException;
import java.net.URI;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.ros.address.InetAddressFactory;
import org.ros.android.AccompanyRosApp;
import org.ros.android.NodeMainExecutorService;
import org.ros.node.NodeConfiguration;


import android.app.ProgressDialog;
import android.content.Context;
import android.graphics.Bitmap;
import android.hardware.usb.UsbManager;
import android.os.Handler;
import android.util.Log;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

public class AccompanyGUIApp extends AccompanyRosApp{
	
	public final String TAG="Accompany GUI - Application";
	
	//Activities identifiers
	public final int USER_VIEW=1;
	public final int ROBOT_VIEW=2;
	public final int ACTIONS_VIEW=3;
	public final int EXECUTE_VIEW=4;
	public final int LOGIN=5;
	public final int SETTINGS=6;
	
	//DB reqeusts identifiers
	public final int ALL_ACTIONS_REQUEST_CODE=1;
	public final int USER_ACTIONS_REQUEST_CODE=2;
	public final int ROBOT_ACTIONS_REQUEST_CODE=3;
	public final int EXPRESSION_REQUEST=5;
	
	//squeeze me and cal me action id
	public final int COMEHEREID=518;
		
	//Ros stuffs
	protected String RosMasterIP;  // ros master ip address
	
	//The user id saved after login (don't touch it!!)
	protected int userId=-1;
	protected int userLang=-1;
		
	//the running activity
	protected int running=0;
	protected int old_running;
	protected boolean isShowingSettings=false;
	protected int on_last_options_request_running;
		
	//the app's activities
	protected UserView  user_act  = null;
	protected RobotView robot_act = null;
	protected RobotWorkingView robot_working_view= null;
	protected ActionsListView actions_list= null;
	protected LoginPage login_page = null;
	
	//handlers
	protected Handler db_handler;         //database client handler
	protected Handler ControllerHandler;  //the handler for the thread that monitors the robot's status
	protected Handler images_handler;     //the handler for the upgrade of images coming from robot
	protected Handler actions_handler;    //the handler to send the actions from Call-me or squeeze-me modules
	protected Handler expression_handler;  //the handler to change expression on screen
	
	//ros nodes
	public CmdVelocityPublisher turn_talker;
	public HeadControllerGUI head_controller; 
	public TorsoControllerGUI torso_controller;
	protected ImagesSubscriber<sensor_msgs.CompressedImage> imagesSubscriber;
	public EmphasisClient emp_client;
	
	//app's Threads
	public SpeechThread st=null;                    //Thread that implements the call me functionality
	protected RobotStatusThread StatusController;   //Thread that monitors the status of the robot
	protected MaskExpressionThread met;             //Thread controlling the current robot emotion
	protected ActionPossibilitiesUpdateThread apt;  //Thread refreshing the state of APs
	
	//app's clients
	public DatabaseClient db_client;
	
	//speech state (tracked to return to the original one after an executed action)
	protected boolean oldSpeechState;
	
	/*********************************/
	
	//current image from robot
	protected Bitmap robot_image;
	
	//screen dimesions:
	int dw;
	int dh;
	

	@Override
	public void onCreate()
	{
		super.onCreate();
		running=-1;
	}
	
	@Override
	public void closeApp() {
		if (user_act  != null) user_act.halt();
		if (robot_act != null) robot_act.halt();
		if (robot_working_view != null) robot_working_view.halt();
		if (actions_list != null) actions_list.halt();
		if (login_page!= null) login_page.finish();
		if (st!=null) st.halt();
		if (met!=null) met.halt();
		closeNodeMainExecutorService();
		if (robot_image!=null) robot_image.recycle();
		robot_image=null;
		
		 stopIoManager();
	     if (mSerialDevice != null) {
	         try {
	        	 mSerialDevice.close();
	         } catch (IOException e) {
	             // Ignore.
	         }
	         mSerialDevice = null;
	     }
	     
	     //if (oms!=null) oms.stopSelf();
		
	     if (StatusController!=null) StatusController.halt();
	     if (met!=null) met.halt();
	     if (apt!=null) apt.halt();
	     
		System.gc();
		android.os.Process.killProcess(android.os.Process.myPid());
	}
	
	public void closeAppOnError(final String msg)
	{
		switch (running)
		{
			case ROBOT_VIEW:
			{
				ControllerHandler.post(new Runnable(){

					@Override
					public void run() {
						final ProgressDialog pdd= ProgressDialog.show(robot_act, robot_act.getResources().getString(R.string.gui_title), msg);
						pdd.setIcon(android.R.drawable.ic_dialog_alert);
						Thread waiter = new Thread() {
							@Override
							public void run()
							{
								final int max_time=2000;  //5 secondi di attesa del servizo
								int waited=0;
								try{
									while (waited < max_time)
									{
										Thread.sleep(100);
										waited+=100;
									}
								} catch(final Exception e){
									
								}
								finally{  //if service is active send request to db
									pdd.dismiss();
									AccompanyGUIApp.this.closeApp();
								}
							}
						};
						waiter.start();
					}});
			} break;
			case USER_VIEW:
			{
				ControllerHandler.post(new Runnable(){

					@Override
					public void run() {
						final ProgressDialog pdd= ProgressDialog.show(user_act, user_act.getResources().getString(R.string.gui_title), msg);
						pdd.setIcon(android.R.drawable.ic_dialog_alert);
						Thread waiter = new Thread() {
							@Override
							public void run()
							{
								final int max_time=2000;  //5 secondi di attesa del servizo
								int waited=0;
								try{
									while (waited < max_time)
									{
										Thread.sleep(100);
										waited+=100;
									}
								} catch(final Exception e){
									
								}
								finally{  //if service is active send request to db
									pdd.dismiss();
									AccompanyGUIApp.this.closeApp();
								}
							}
						};
						waiter.start();
					}});
			} break;
			case ACTIONS_VIEW:
			{
				//actions_list.toastMessage(phrase);
				closeApp();
			} break;
			case EXECUTE_VIEW:
			{
				final ProgressDialog pdd= ProgressDialog.show(robot_working_view, robot_working_view.getResources().getString(R.string.gui_title), msg);
				pdd.setIcon(android.R.drawable.ic_dialog_alert);
				Thread waiter = new Thread() {
					@Override
					public void run()
					{
						final int max_time=2000;  //5 secondi di attesa del servizo
						int waited=0;
						try{
							while (waited < max_time)
							{
								Thread.sleep(100);
								waited+=100;
							}
						} catch(final Exception e){
							
						}
						finally{  //if service is active send request to db
							pdd.dismiss();
							AccompanyGUIApp.this.closeApp();
						}
					}
				};
				waiter.start();
			}
		}
	}

	@Override
	public void init(NodeMainExecutorService nmes) {
		//crea il thread di controllo dello stato
		StatusController.start();
		apt.start();
		//create the Thread for Call-me module
		if ((st==null)) 
		{
			st= new SpeechThread(this,oldSpeechState,actions_handler);
			st.start();
		}
		//create the Thread for the expression mamagement
		if (met==null)
		{
			met=new MaskExpressionThread(actions_handler,db_client,this);
			met.start();
		}
		
		//try to start the ros nodes
		try{
			turn_talker= new CmdVelocityPublisher();
			head_controller = new HeadControllerGUI(); 
			torso_controller = new TorsoControllerGUI();
			imagesSubscriber= new ImagesSubscriber<sensor_msgs.CompressedImage>(images_handler,this);
			imagesSubscriber.setTopicName("accompany/GUIimage/compressed");
		    imagesSubscriber.setMessageType(sensor_msgs.CompressedImage._TYPE);
		    emp_client= new EmphasisClient(this);
		    
		    NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),
				    new URI("http://"+RosMasterIP+":11311/"));
			nmes.execute(imagesSubscriber, nodeConfiguration.setNodeName("AccompanyGUI/RobotView_image"));
			nmes.execute(turn_talker, nodeConfiguration.setNodeName("AccompanyGUI/CmdVelPub"));
			nmes.execute(head_controller, nodeConfiguration.setNodeName("AccompanyGUI/HeadControllerGUI"));
			nmes.execute(torso_controller, nodeConfiguration.setNodeName("AccompanyGUI/TorsoControllerGUI"));
			nmes.execute(emp_client, nodeConfiguration.setNodeName("AccompanyGUI/emphasis_client"));
			Log.e(TAG,"ROS services started");	
		}catch(Exception e)
		{
			Log.e(TAG,"GRAVE: error starting ROS subscribers and publishers!!");
		}
	}
	
	//set the login page to the app
	public void setLoginPage(LoginPage lp)
	{
		running = LOGIN;
		AccompanyPreferences pref= new AccompanyPreferences(lp);
		pref.loadPreferences();
		oldSpeechState=pref.getSpeechMode();
		this.login_page=lp;
		dh= login_page.getDisplayHeight();
		dw= login_page.getDisplayWidth();
		mUsbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
		flag=false;
		Log.e(TAG,"start login page");
		
		db_handler= new Handler();
		db_client= new DatabaseClient(this,db_handler,pref.getDatabaseIp(),pref.getDatabasePort());
		
		if(StatusController==null)
		{
			ControllerHandler= new Handler(); 
			StatusController= new RobotStatusThread(this, ControllerHandler,pref.getDatabaseIp(),pref.getRobotStatusControlPort());  //prima era nell'init
		}
		if(apt==null)
		{
			apt= new ActionPossibilitiesUpdateThread(this,db_handler);
		}
	}
	
	//set the user page to the app
	public void setUserView(UserView u)
	{
		this.user_act=u;
	}
	
	//set the action list page
	public void setActionsView(ActionsListView a)
	{
		actions_list=a;
	}
	
	//set the robot working activity
	public void setRobotWorkingView(RobotWorkingView r)
	{
		this.robot_working_view=r;
	}
	
	//set the robot perspective activity
	public void setRobotView(RobotView r)
	{
		this.robot_act=r;
	}
	
	//userId management
	public void setUserId(int uid)
	{
		userId=uid;
	}
	
	public int getUid()
	{
		return userId;
	}
	
	//management of running activity
	public void setRunningActivity(int r)
	{
		running=r;
	}
	
	//returns the running activity code
	public int getRunning()
	{
		return running;
	}
	
	public void robotBusy()
	{
		old_running=running;
		running= EXECUTE_VIEW;
	}
	
	
	public void SetIp(String mip)
	{
		RosMasterIP=mip;
	}
	
	public void startServices()
	{
		setURI(RosMasterIP);
		images_handler= new Handler();
		expression_handler = new Handler();
		actions_handler = new Handler(); 
		startNodeMainExecutorService();
	}
	
	public void startServices(String ip,int r)
	{
		RosMasterIP=ip;
		setURI(RosMasterIP);
		images_handler= new Handler();
		expression_handler = new Handler();
		actions_handler = new Handler(); 
		startNodeMainExecutorService();
	}
	
	//MANAGEMENT OF SETTINGS ACTIVITY
	public void setSettings()
	{
		isShowingSettings=true;
	}
	
	public void unsetSettings()
	{
		isShowingSettings=false;
	}
	
	//ROBOT STATUS MONITORING
	
	public String getCurrentTask()
	{
	 	return StatusController.getCurrentTask();
	}
	
	//EXPRESSION UTILITIES
	
	 public int getCurrentExpression()
	 {
	   	return met.getCurrentExpression();
	 }
	
	//IMAGES MANAGEMENT
	
	public void StartSubscribing()
	{
		imagesSubscriber.startSubscribing();
	}
	
	public void stopSubscribing()
	{
		imagesSubscriber.stopSubscribing();
	}
	
	public Bitmap getLastImage()
	{
		return robot_image;
	}
	
	 public void setBitmap(Bitmap b)
	 {
		 if (b!=null)
			{
				if (robot_image!=null) robot_image.recycle();
				robot_image=b;

				if (running==this.ROBOT_VIEW)
				{
					robot_act.refreshImage(robot_image);//,m);
				}
				if (running==this.EXECUTE_VIEW)
				{
					if (robot_working_view==null)
						Log.e(TAG,"error trying to publish in a null activity (executing)");
					else
						robot_working_view.refreshImage(robot_image);//,m);
				}
			}    	
	 }
	
	/******************************************************************/
	/*                    CALL-ME MODULE MANAGEMENT                   */
	/******************************************************************/
	
	//speech status management
	public void setOldSpeech(boolean b)
	{
		oldSpeechState=b;
	}
		
	public boolean getOldSpeech()
	{
		return oldSpeechState;
	}
	
	public void startSpeechRecognition()
	{
		Log.e(TAG,"Speech start: Call me active");
		oldSpeechState=true;
		st.setMode(oldSpeechState);
	}
	
	public void stopSpeechrecognition()
	{
		Log.e(TAG,"Speech stop - Call me inactive");
		oldSpeechState=false;
		st.setMode(oldSpeechState);
	}
	
	/******************************************************************/
	/*                 DB REQUESTS AND RESPONSES MANAGEMENT           */
	/******************************************************************/
	
	//options request from an Ap
	 public void requestOptions(int id)
	 {
		on_last_options_request_running=running;
	 	db_client.requestOptions(id, Integer.toString(userLang));
	 }
	 
	public void ThreadRequest(int i)
	{
		if (i==USER_ACTIONS_REQUEST_CODE) db_client.thread_request(i, Integer.toString(userId)
				,Integer.toString(userLang));
			else if (i == ROBOT_ACTIONS_REQUEST_CODE) db_client.thread_request(i, Integer.toString(userId),Integer.toString(userLang));
	}

	public void RequestToDB(int i)//, int son_id)
	{
		if (i==USER_ACTIONS_REQUEST_CODE) db_client.request(i, Integer.toString(userId)
				,Integer.toString(userLang));
			else if (i == ROBOT_ACTIONS_REQUEST_CODE) db_client.request(i, Integer.toString(userId),Integer.toString(userLang));
	}
	
	public void sendActionRequest(int id)
	{
		db_client.sendCommand(id);
	}
	 
	 
	//Handle the login answer and save user id and language
	public void handleLoginResponse(String a,int i1,int i2)
	{
		startServices();
		this.userId= i1;
		this.userLang=i2;
		login_page.loginResult(a);
	}
	
	//full action list response
	public void handleFullActionsListResponse(String response)
	{
		Log.i("app-response","res: "+response+", required from: full actions list, running: "+running);
		if (running==ACTIONS_VIEW)
			actions_list.handleResponse(response);
		
		if (oldSpeechState) startSpeechRecognition();
		//reset the standard robot speed
		emp_client.setEmphasis(0.3);
	}
	
	public void handleResponse(String response,int code)
	{
		Log.i("app-response","res: "+response+", code: "+code+", "+"running: "+running);

		if ((running==USER_VIEW)&&(code==USER_ACTIONS_REQUEST_CODE))
			user_act.handleResponse(response);
		if ((running==ROBOT_VIEW)&&(code==ROBOT_ACTIONS_REQUEST_CODE))
			robot_act.handleResponse(response);
		
		if (oldSpeechState) startSpeechRecognition();
		//reset the standard robot speed
		//if (emp_client!=null) emp_client.setEmphasis(0.3);
	}
	
	public void handleThreadResponse(String response,int code)
	{
		Log.i("app-response","res: "+response+", code: "+code+", "+"running: "+running);
		
		
		
		if (oldSpeechState) startSpeechRecognition();
	}
	
	 public void handleOptionsResponse(String response,int id)
	 {
			Log.i("app-response","res: "+response+", required from: options, running: "+running);
			//if ((running==ACTIONS_VIEW)&&(on_last_options_request_running==running))
			//	actions_list.handleOptionsResponse(response,id);
			if ((running==USER_VIEW)&&(on_last_options_request_running==running))
				user_act.handleOptionsResponse(response,id);
			if ((running==ROBOT_VIEW)&&(on_last_options_request_running==running))
				robot_act.handleOptionsResponse(response,id);
	 }
	
	public void handleExpressionResponse(String response)
	{
		met.handleResponse(response);
	}
	
	public void handleActionResponse()
	{
		robotFree();
	}
	
	public void handleFailedActionResponse()
	{
		Log.e(TAG,"switching back to "+old_running);
		if (running!=EXECUTE_VIEW) Log.e(	TAG,"ack already received");
		else
			switch (old_running)
			{
				case ROBOT_VIEW:
				{
					robot_working_view.switchToRobotView();
					robot_act.toastMessage("command failed!");
					if (oldSpeechState) startSpeechRecognition();
				} break;
				case USER_VIEW:
				{
					robot_working_view.switchToUserView();
					user_act.toastMessage("command failed!");
					if (oldSpeechState) startSpeechRecognition();
				} break;
				case ACTIONS_VIEW:
				{
					robot_working_view.switchToActionsList();
					actions_list.toastMessage("command failed!");
					if (oldSpeechState) startSpeechRecognition();
				} break;
				case EXECUTE_VIEW:
				{
					Log.e(TAG,"GRAVE: old_running must never be equal to EXECUTE_VIEW!!!");
				}
			}
		//reset the standard robot speed
		emp_client.setEmphasis(0.3);
	}
	
	//to respond to a command executed situation
	public void robotFree()
	{
		Log.e(TAG,"Robot free: switching back to "+old_running);
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
		
		//reset the emphasis
		if (emp_client!=null) emp_client.setEmphasis(0.3);
	}
	
	public void showRobotExecutingCommandView()
	{
		unsetSettings();
		
		if (st!=null) st.setMode(false);
		switch (running)
		{
			case ROBOT_VIEW:
			{
				robot_act.showRobotExecutingCommandView();
			} break;
			case USER_VIEW:
			{
				user_act.showRobotExecutingCommandView();
			} break;
			case ACTIONS_VIEW:
			{
				actions_list.showRobotExecutingCommandView();
			} break;
			case EXECUTE_VIEW:
			{
				robot_working_view.changeAction();
				//Log.e("AccompanyGUI","GRAVE: running must never be equal to EXECUTE_VIEW when you are sending a command!!!");
			}
		}
	}
	
	public void switchMask(int before, int after)
    {
    	if (running==ROBOT_VIEW)
    	{
    		robot_act.switchMask(before,after);
    	}
    	else if (running==EXECUTE_VIEW)
    		{
    			robot_working_view.switchMask(before,after);
    		}
    }
	
	/******************************************************************/
	/*         PART FOR THE SQUEEZE-ME MODULE USB MANAGEMENT          */
	/******************************************************************/
	
	/**
     * The system's USB service.
     */
    public UsbManager mUsbManager;
    
    public UsbSerialDriver mSerialDevice;
    private final ExecutorService mExecutor = Executors.newSingleThreadExecutor();

    private SerialInputOutputManager mSerialIoManager;
    
    //squeeze computation:
  	protected int UPPER_THR_EMPHASIS=35;
  	protected int LOWER_THR_EMPHASIS=20;
  	protected int emphasis;
  	protected int field1;
  	protected int field2;
  	protected long time;
    
    //message management
    private String message;
    private String myM;
    private boolean flag;

    private final SerialInputOutputManager.Listener mListener =
            new SerialInputOutputManager.Listener() {

        @Override
        public void onRunError(Exception e) {
            Log.d(TAG, "Squeeze-me: Runner stopped.");
        }

        @Override
        public void onNewData(final byte[] data) {
            //MainActivity.this.runOnUiThread(new Runnable() {
            //    @Override
            //    public void run() {
                    updateReceivedData(data);
            //    }
            //});
        }
    };
    
    private void updateReceivedData(byte[] data){
   	
       message=new String(data);
       Log.e(TAG+" (squeeze)","Squeeze-me message: "+message);
       if (!flag ) myM = "";
       int i=0;
       int start=0;
       if (!flag)
       {
       	start=message.indexOf("^");
       	flag=true;
       }
       int end=-1;
       if (message.contains("$"))
       	end=message.indexOf("$");
       if (end!=-1)
       {
       	myM=myM+=message.substring(start, end+1);
       	flag=false;
       	myM=myM.replace("$", "");
       	myM=myM.replace("^", "");
       	String[] split=myM.split(" ");
    	    Log.e(TAG+" (squeeze)","f1: "+split[0]);
    	    Log.e(TAG+" (squeeze)","f2: "+split[1]);
    	    Log.e(TAG+" (squeeze)","emp: "+split[2]);
    	    emphasis= Integer.parseInt(split[2]);
    	    field1=Integer.parseInt(split[0]);
    	    field2=Integer.parseInt(split[1]);
    	    //if ((emphasis>UPPER_THR_EMPHASIS)&&(running!=-1)&&(running!=EXECUTE_VIEW))
    	    if ((emphasis>UPPER_THR_EMPHASIS)&&(running!=-1))
    	    {
    	    	emp_client.setEmphasis(((double)(emphasis-10))/100);
    	    	if (running!=EXECUTE_VIEW)
	    		{
    	    		actions_handler.post(new Runnable()
    	    		{
    	    			@Override
    	    			public void run() {
    	    				toastMessage("I'm coming");
    	    			}});
    	    	  	//sendActionRequest("come_here","I'm coming");
    	    		sendActionRequest(COMEHEREID);
	    		}
    	    }
    	    else
    	    	if ((emphasis>LOWER_THR_EMPHASIS)&&(running!=-1))
    	    	//if ((emphasis>LOWER_THR_EMPHASIS)&&(running!=-1)&&(running!=EXECUTE_VIEW))	
    	    	{
    	    		emp_client.setEmphasis(((double)(emphasis-10))/100);
    	    		if (running!=EXECUTE_VIEW)
    	    		{
    	    			actions_handler.post(new Runnable()
    	    			{
    	    				@Override
    	    				public void run() {
    	    					toastMessage("I'm coming");
    	    				}});
    	    			//sendActionRequest("come_here","I'm coming");
    	    			sendActionRequest(COMEHEREID);
    	    		}
    	    	}
       }
       else
       {
       	myM+=message.substring(start);
       }
   }
    
    private void stopIoManager() {
        if (mSerialIoManager != null) {
            Log.i(TAG+" (squeeze)", "Stopping io manager ..");
            mSerialIoManager.stop();
            mSerialIoManager = null;
        }
    }

    private void startIoManager() {
        if (mSerialDevice != null) {
            Log.i(TAG+" (squeeze)", "Starting io manager ..");
            mSerialIoManager = new SerialInputOutputManager(mSerialDevice, mListener);
            mExecutor.submit(mSerialIoManager);
        }
    }

    public void onDeviceStateChange() {
        stopIoManager();
        startIoManager();
    }
    
    /******************************************************************/
	/*                        UTILITY PART                            */
	/******************************************************************/
    
    //to send a toast message wiothout knowing the running activity
    public void toastMessage(String phrase)
	{
    	Log.i(TAG,"asdfghjkl");
		switch (running)
		{
			case ROBOT_VIEW:
			{
				robot_act.toastMessage(phrase);
			} break;
			case USER_VIEW:
			{
				user_act.toastMessage(phrase);
			} break;
			case ACTIONS_VIEW:
			{
				actions_list.toastMessage(phrase);
			} break;
			case EXECUTE_VIEW:
			{
				robot_working_view.toastMessage(phrase);
			}break;
			case LOGIN:
			{
				login_page.toastMessage(phrase);
			}break;
			default:
			{
				login_page.toastMessage(phrase);
			}break;
		}
	}
    
    public void updatePreferences()
    {
    	AccompanyPreferences pref;
    	switch (running)
		{
			case ROBOT_VIEW:
			{
				pref= new AccompanyPreferences(robot_act);
;			} break;
			case USER_VIEW:
			{
				pref= new AccompanyPreferences(user_act);
			} break;
			case ACTIONS_VIEW:
			{
				pref= new AccompanyPreferences(actions_list);
			} break;
			case EXECUTE_VIEW:
			{
				pref= new AccompanyPreferences(robot_working_view);
			}
			case LOGIN:
			{
				pref= new AccompanyPreferences(login_page);
			} break;
			default:
			{
				pref= new AccompanyPreferences(login_page);
			} break;
		}
    	pref.loadPreferences();
    	db_client.createBaseUrl(pref.getDatabaseIp(), pref.getDatabasePort());
    	StatusController.createBaseUrl(pref.getDatabaseIp(),pref.getRobotStatusControlPort());
    }
}

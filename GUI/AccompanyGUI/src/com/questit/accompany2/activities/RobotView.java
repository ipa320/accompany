package com.questit.accompany2.activities;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;

import org.apache.http.HttpEntity;
import org.apache.http.HttpResponse;
import org.apache.http.NameValuePair;
import org.apache.http.client.HttpClient;
import org.apache.http.client.entity.UrlEncodedFormEntity;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.message.BasicNameValuePair;
import org.apache.http.params.BasicHttpParams;
import org.apache.http.params.HttpConnectionParams;
import org.apache.http.params.HttpParams;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.MyRosActivity;
import org.ros.android.view.RosImageViewReversed;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Rect;
import android.os.Bundle;
import android.os.StrictMode;
import android.util.Log;
import android.view.Display;
import android.view.GestureDetector;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.widget.AbsoluteLayout;
import android.widget.Button;
import android.widget.FrameLayout.LayoutParams;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.PopupWindow;
import android.widget.TextView;
import android.widget.Toast;

import com.questit.accompany2.AccompanyGUI_app2;
import com.questit.accompany2.AccompanyPreferences;
import com.questit.accompany2.R;
import com.questit.accompany2.RosNodes.CmdVelocityPublisher;
import com.questit.accompany2.RosNodes.HeadControllerGUI;
import com.questit.accompany2.widget.RobotFatherLabel;
import com.questit.accompany2.widget.RobotSimpleLabel;
import com.questit.accompany2.widget.RobotSonLabel;
import org.ros.rosjava_tutorial_pubsub.Talker;

public class RobotView extends Activity{	
	
	protected final int SETTINGS_CODE=10;
	
	protected AccompanyGUI_app2 myApp;
	protected RobotView me;
	
	
		protected Matrix my_matrix;
	//protected CmdVelocityPublisher turn_talker;//(for turning the robot around with touch)
	//protected HeadControllerGUI head_controller;
	
	protected ImageButton left_button;
	protected ImageButton right_button;
	protected AbsoluteLayout my_layout;
	protected ImageView image;
	//protected ImageView mask;
	
	//the labels:
	protected ArrayList<RobotSimpleLabel> SimpleLabels;
	protected ArrayList<RobotFatherLabel> FatherLabels;
	protected ArrayList<RobotSonLabel> SonLabels;
	
	protected AccompanyPreferences myPreferences;
	
	protected double cam_position=-3.1415926;
	protected double torso_position=0;
	
	protected GestureDetector g = null;
	protected View.OnTouchListener gl;
	
	protected PopupWindow popupWindow;
	
	//action detection
	boolean flag_move;
	int myLastTouch_x;
	int myLastTouch_y;
	int pointerId;
	private final int INVALID_POINTER_ID=-1;
	private final int PIXEL_TRESHOLD_GESTURE=5;
	
	/*public RobotView()
	{
		super("RobotView","RobotView");
	}
	
	protected RobotView(String notificationTicker, String notificationTitle) 
	{
		super("RobotView", "RobotView");
	}*/
	
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		//standrd things
		super.onCreate(savedInstanceState);
		this.setContentView(R.layout.robot_view);
		
		Log.i("INFO","Robot View on create!!");
		
		//recovering the application that owns this activity
		//and setting the activity variable
		myApp=(AccompanyGUI_app2)this.getApplication();
		myApp.setRobotView(this);
		me=this;
		//bring camera to front
		myApp.head_controller.bringCameraToFront();
		myApp.torso_controller.bringHome();
		
		//reading the preferences 
		myPreferences=new AccompanyPreferences(this);
		myPreferences.loadPreferences();
		
		Log.i("AccompanyGUI","on create robotView");
		myApp.setRunningActivity(myApp.ROBOT_VIEW);
		
		 //Setting up the policies for the use of threads
        if (android.os.Build.VERSION.SDK_INT > 9) {
			StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder()
					.permitAll().build();
			StrictMode.setThreadPolicy(policy);
		}
		
		left_button=(ImageButton)this.findViewById(R.id.left_robot_button);
		right_button=(ImageButton)this.findViewById(R.id.right_robot_button);
		my_layout=(AbsoluteLayout)this.findViewById(R.id.robot_layout);
		image=(ImageView)this.findViewById(R.id.robot_standard_image_view);
		image.setImageBitmap(myApp.getLastImage());
		/*image.setScreenDimensions(this.getDisplayWidth(),this.getDisplayHeight());
	    image.setTopicName("/stereo/left/image_color/compressed");
	    image.setMessageType(sensor_msgs.CompressedImage._TYPE);
	    image.setMessageToBitmapCallable(new BitmapFromCompressedImage());*/
		//Setting the listners for the banners:
		left_button.setOnClickListener(new View.OnClickListener() {			
			@Override
			public void onClick(View v) {
				switchToUserView();
			}		});	
		right_button.setOnClickListener(new View.OnClickListener() {			
			@Override
			public void onClick(View v) {
				switchToUserView();
			}		});	
		
		SimpleLabels= new ArrayList<RobotSimpleLabel>();
        FatherLabels= new ArrayList<RobotFatherLabel>();
        SonLabels= new ArrayList<RobotSonLabel>();
		
        //set up gesture detection
        //g=new GestureDetector(new myGestureDetector(myPreferences.getRobotIP(),myPreferences.getRobotPort(),this));
        gl = new View.OnTouchListener() {
			@Override
			public boolean onTouch(View v, MotionEvent event) {
				/*if (g.onTouchEvent(event)) {
                    return false;
                }
                return true;*/
				AbsoluteLayout myView=(AbsoluteLayout)v;
				
				
				final int action=event.getAction();
				switch (action & MotionEvent.ACTION_MASK)
				{
				case MotionEvent.ACTION_DOWN:
				{
					Log.d("Accompany-gestureDet","Action down");
					flag_move=false;
					myLastTouch_x=(int)event.getX();
					myLastTouch_y=(int)event.getY();
					pointerId=event.getPointerId(0);
					//remove labels (e banners se possibile)
					my_layout.removeAllViews();
					break;
				}
				case MotionEvent.ACTION_MOVE:
				{
					Log.d("Accompany-gestureDet","Action move");
					flag_move=true;
					final int act_p=event.getPointerId(pointerId);
					final float x= event.getX();
					final float y= event.getY();
					final float dx= -(x-myLastTouch_x);
					float dy= -(y-myLastTouch_y);
					//move! (send message on topic!!)
					if (Math.abs(dx) > PIXEL_TRESHOLD_GESTURE)
						myApp.turn_talker.publish(-(float)(dx*Math.PI/180)/2);                   //Turn base!
					Log.i("pub","dy: "+dy);
					//camera management:
					/*if (dy<0)
					{
						double diff=myApp.head_controller.publish((dy*Math.PI/180)/10);
						if (diff!=0) myApp.torso_controller.publish(diff);
					}
					else
					{
						double diff=myApp.head_controller.publish((dy*Math.PI/180)/10);
					}
					//{
					//	if (myApp.torso_controller)
					//}
					/*if ((-0.1<myApp.torso_controller.getDesiredTorsoPos()[0])&&(myApp.torso_controller.getDesiredTorsoPos()[0]<0.1))
						
					if (myApp.head_controller.getDesiredHeadPos()==myApp.head_controller.max_head_pos)
					{
						
					}
					else if (myApp.head_controller.getDesiredHeadPos()==myApp.head_controller.min_head_pos)
					{
						
					}
					else*/
					Log.i("head pos",myApp.head_controller.getHeadPos()+"");
					if (Math.abs(dy) > PIXEL_TRESHOLD_GESTURE)
					{
						//if (myApp.head_controller.getHeadPos()<-(myApp.head_controller.max_head_pos-myApp.head_controller.min_head_pos)/2)
						//	dy=-dy;
						double diff=myApp.head_controller.publish((dy*Math.PI/180)/10);   //Turn Head!
						Log.i("head diff",diff+"");
						if (diff!=0) myApp.torso_controller.publish(diff);
					}
					
					myLastTouch_x=(int)x;
					myLastTouch_y=(int)y;
					break;
				}
				case MotionEvent.ACTION_UP:
				{
					Log.d("Accompany-gestureDet","Action up");
					// show labels again
					//getRobotLabelsFromDB();
					//addLabels(my_layout);
					if (isClientDbWorking()) sendRequest();
					
					
					pointerId= INVALID_POINTER_ID;
					break;
				}
				case MotionEvent.ACTION_CANCEL:
				{
					Log.d("Accompany-gestureDet","Action cancel");
					// show labels again
					//getRobotLabelsFromDB();
					//addLabels(my_layout);
					if (isClientDbWorking()) sendRequest();
					
					pointerId= INVALID_POINTER_ID;
					break;
				}
				case MotionEvent.ACTION_POINTER_UP:
				{
					Log.d("Accompany-gestureDet","Action pointer up");
					final int p_idx=(event.getAction() & MotionEvent.ACTION_POINTER_INDEX_MASK) >> 
					MotionEvent.ACTION_POINTER_INDEX_SHIFT;
					final int ppppp= event.getPointerId(p_idx);
					if (ppppp==pointerId)
					{
						final int newPointer = p_idx == 0 ? 1 : 0;
						myLastTouch_x=(int) event.getX();
						myLastTouch_y=(int) event.getY();
						pointerId=event.getPointerId(newPointer);
					}			
					break;
				}
				}
				return true;
			}
        };
        my_layout.setOnTouchListener(gl);
        
		//getRobotLabelsFromDB();
		//addLabels(my_layout);  //se no fallo su imageView
		if (isClientDbWorking()) sendRequest();
	}
	
	public void sendRobotActionRequest(String command, String phrase)
	{
		myApp.sendActionRequest(command, phrase);
	}
	
	@Override
	public void onRestart()
	{
		Log.i("AccompanyGUI","on restart robotView");
		myPreferences.loadPreferences();
		super.onRestart();
		//getting the robot action possibilities from the DB (it includes the removal of the old labels!):
		//getRobotLabelsFromDB();
		//showing them in the layout
		//addLabels(my_layout);
		if (isClientDbWorking()) sendRequest();
		//SETTING THIS AS THE ACTIVITY FOCUSED
		myApp.setRunningActivity(myApp.ROBOT_VIEW);
	}
	
	public void sendRequest()
	{
		myApp.RequestToDB(myApp.ROBOT_ACTIONS_REQUEST_CODE,-1);
	}
	
	protected int last_father_ap_id=-1;
	RobotFatherLabel lastFatherClicked;
	
	public void sendSonRequest(int id,RobotFatherLabel rfl)
	{
		last_father_ap_id=id;
		lastFatherClicked=rfl;
		myApp.RequestToDB(myApp.SONS_REQUEST_CODE, id);
	}
	
	public void handleResponse(String res)
	{
		//remove all labels from layout:
    	removeAllLabels();
    	//rereating labels lists
        SimpleLabels= new ArrayList<RobotSimpleLabel>();
        FatherLabels= new ArrayList<RobotFatherLabel>();
        SonLabels= new ArrayList<RobotSonLabel>();
		//parse json data
        try{
            JSONArray jArray = new JSONArray(res);
            for(int i=0;i<jArray.length();i++){
                JSONObject json_data = jArray.getJSONObject(i);
                //Log.i("readen","Type: "+json_data.getInt("Type")+", Name: "+json_data.getString("Name"));
                //OnOffLabel ool=new OnOffLabel(getApplicationContext(),json_data.getString("Name"),CareOBotIP);
                if ((json_data.getString("type_description").contains("Simple"))||
                		(json_data.getString("type_description").equals("simple")))
                {
                	//spawn simple label
                	RobotSimpleLabel sl= new RobotSimpleLabel(getApplicationContext(),
                			json_data.getString("ap_label"),json_data.getString("command"),
                			json_data.getString("phraseal_feedback"),json_data.getDouble("likelihood"),
                			0,0,my_layout,this);
                	SimpleLabels.add(sl);
                }
                else
                	if((json_data.getString("type_description").contains("father"))||
                			(json_data.getString("type_description").contains("parent")))
                	{
                		//spawn father label
                		RobotFatherLabel fl= new RobotFatherLabel(getApplicationContext(),
                				json_data.getInt("apId"),
                				json_data.getString("ap_label"),
                				json_data.getString("command"),
                				json_data.getDouble("likelihood"),
                				0,0,my_layout,this);
                		FatherLabels.add(fl);
                	} 
                	else {
                		//spawn seek label
                	}
            }
        }catch(JSONException e){
            Log.e("log_tag", "Error parsing data "+e.toString());
        }
        addLabels(my_layout);
	}
	
	public void handleSonResponse(String res,int son)
	{
		if (last_father_ap_id==son)
			lastFatherClicked.setSons(res);
		lastFatherClicked=null;
		last_father_ap_id=-1;
			
	}
	
	protected boolean isClientDbWorking()
	{
		if (myApp.db_client==null) return false;
		else if (myApp.db_client.isStarted()) return true;
			else return false;
	}
	
	/*protected void getRobotLabelsFromDB()
	{
		//remove all labels from layout:
    	removeAllLabels();
    	//rereating labels lists
        SimpleLabels= new ArrayList<RobotSimpleLabel>();
        FatherLabels= new ArrayList<RobotFatherLabel>();
        SonLabels= new ArrayList<RobotSonLabel>();
    	
    	//this will be recover from the robot in the final version
    	int condition_id=1;
    	
    	InputStream is=null;
        String result="";
        //data to send
        ArrayList<NameValuePair> nameValuePairs = new ArrayList<NameValuePair>();
        //nameValuePairs.add(new BasicNameValuePair("condition","1"));
        nameValuePairs.add(new BasicNameValuePair("location","502"));
        
        //attempt to connect to remote mysql database trought the apposite php web page
        try{
        	//setting timetouts for the connection!!
        	HttpParams httpParameters = new BasicHttpParams();
        	int timeoutConnection = 3000;  //the proper connection timeout
        	HttpConnectionParams.setConnectionTimeout(httpParameters, timeoutConnection);
        	int timeoutSocket = 5000;      //the operation timeout
        	HttpConnectionParams.setSoTimeout(httpParameters, timeoutSocket);
        	//the client with these parameters:
        	HttpClient hc= new DefaultHttpClient(httpParameters); 
        	HttpPost hp= new HttpPost("http://"+myPreferences.getDataBaseIP()+"/robot_labels.php");
        	Log.i("AccompanyGUI","connecting to: http://"+myPreferences.getDataBaseIP()+"/robot_labels.php");
            //HttpPost hp= new HttpPost("http://192.168.1.104/php_labels.php");
        	hp.setEntity(new UrlEncodedFormEntity(nameValuePairs));
        	Log.i("INFO","Starting query");
        	HttpResponse response = hc.execute(hp);
        	Log.i("INFO","Executed the query");
        	HttpEntity entity = response.getEntity();

            is = entity.getContent();
        }
        catch(Exception e){
        	Log.e("ERROR", "Error in http connection to remote mysql db: "+ e.toString());
        }
        //convert response to string
        try{

            BufferedReader reader = new BufferedReader(new InputStreamReader(is,"iso-8859-1"),8);
            StringBuilder sb = new StringBuilder();
            String line = null;
            while ((line = reader.readLine()) != null) {
                sb.append(line + "\n");
            }
            is.close();

            result=sb.toString();
        }catch(Exception e){
            Log.e("ERROR", "Error converting result of http request: "+e.toString());
        }
        
        //parse json data
        try{
            JSONArray jArray = new JSONArray(result);
            for(int i=0;i<jArray.length();i++){
                JSONObject json_data = jArray.getJSONObject(i);
                //Log.i("readen","Type: "+json_data.getInt("Type")+", Name: "+json_data.getString("Name"));
                //OnOffLabel ool=new OnOffLabel(getApplicationContext(),json_data.getString("Name"),CareOBotIP);
                if ((json_data.getString("type_description").contains("Simple"))||
                		(json_data.getString("type_description").equals("simple")))
                {
                	//spawn simple label
                	RobotSimpleLabel sl= new RobotSimpleLabel(getApplicationContext(),
                			json_data.getString("ap_label"),json_data.getString("command"),
                			json_data.getString("phraseal_feedback"),json_data.getDouble("likelihood"),
                			0,0,myPreferences.getRobotIP(),myPreferences.getRobotPort(),my_layout,this);
                	SimpleLabels.add(sl);
                }
                else
                	if((json_data.getString("type_description").contains("father"))||
                			(json_data.getString("type_description").contains("parent")))
                	{
                		//spawn father label
                		RobotFatherLabel fl= new RobotFatherLabel(getApplicationContext(),
                				json_data.getInt("apId"),
                				json_data.getString("ap_label"),
                				json_data.getString("command"),
                				json_data.getDouble("likelihood"),
                				0,0,my_layout,myPreferences.getRobotIP(),this,
                				myPreferences.getRobotPort(),myPreferences.getDataBaseIP());
                		FatherLabels.add(fl);
                	} 
                	else {
                		//spawn seek label
                	}
            }
        }catch(JSONException e){
            Log.e("log_tag", "Error parsing data "+e.toString());
        }
	}*/
	
	@SuppressWarnings("deprecation")
	public void addLabels(View v)
    {
    	final int COLLISION_OFFSET_STEP=5;
    	
    	Display d=getWindowManager().getDefaultDisplay();
    	left_button.measure(d.getWidth(), d.getHeight());
    	Log.i("marco",""+left_button.getMeasuredWidth());
    	if (SimpleLabels!=null)
    	{
    		for (int i=0;i<SimpleLabels.size();i++)
    		{	
    			int original_x;
    			int original_y;
    			int flag_offset_direction=0;
    			//per ora si fa cos�, poi sistemeremo!!
    			//SimpleLabels.get(i).setWidth(pixels_width_labels);
    			SimpleLabels.get(i).measure(d.getWidth(), d.getHeight());
    			int xx=original_x=left_button.getMeasuredWidth()+
    					(int)(Math.random()*(d.getWidth()-2*left_button.getMeasuredWidth()-SimpleLabels.get(i).getMeasuredWidth()));
    			int yy=original_y=(int)(Math.random()*(d.getHeight()-SimpleLabels.get(i).getMeasuredHeight()));
    			boolean flag;
    			do
    			{
    				flag=false;
    				SimpleLabels.get(i).setPosition(xx, yy);
    				//AbsoluteLayout.LayoutParams p=Labels.get(i).randomPose(700,400);
    				//SimpleLabels.get(i).autoSetParams(pixels_width_labels);
    				SimpleLabels.get(i).autoSetParams();
    				my_layout.addView(SimpleLabels.get(i));
    				//Log.i("MARCo",SimpleLabels.get(i).getX()+" "+SimpleLabels.get(i).getY());
    				//collision detection:
    				Rect r1;
    				Rect r2;
    				SimpleLabels.get(i).measure(d.getWidth(), d.getHeight());
					int x1=((AbsoluteLayout.LayoutParams)SimpleLabels.get(i).getLayoutParams()).x;
					int y1=((AbsoluteLayout.LayoutParams)SimpleLabels.get(i).getLayoutParams()).y;
					r1= new Rect(x1,y1,(x1+SimpleLabels.get(i).getMeasuredWidth()),(y1+SimpleLabels.get(i).getMeasuredHeight()));
    				for (int h=0;h<i;h++)
    				{
    					SimpleLabels.get(h).measure(d.getWidth(), d.getHeight());
    					int x2=((AbsoluteLayout.LayoutParams)SimpleLabels.get(h).getLayoutParams()).x;
    					int y2=((AbsoluteLayout.LayoutParams)SimpleLabels.get(h).getLayoutParams()).y;
    					r2= new Rect(x2,y2,(x2+SimpleLabels.get(h).getMeasuredWidth()),(y2+SimpleLabels.get(h).getMeasuredHeight()));
    					if (Rect.intersects(r1, r2)) 
						{
							flag= true;
							Log.i("Info","rectangles "+i+" and "+h+" intersecate!");
						}
    				}
    				
    				if (flag) 
					{
						my_layout.removeView(SimpleLabels.get(i));
						if ((flag_offset_direction==0)||(flag_offset_direction==2)) xx+=COLLISION_OFFSET_STEP;
						else  xx-=COLLISION_OFFSET_STEP;
						if ((flag_offset_direction==0)||(flag_offset_direction==1)) yy+=COLLISION_OFFSET_STEP;
						else yy-=COLLISION_OFFSET_STEP;
						if ((yy+SimpleLabels.get(i).getMeasuredHeight())>d.getHeight())
						{
							if (flag_offset_direction==0) flag_offset_direction=2;
							else flag_offset_direction=3;
						}
						if ((xx+SimpleLabels.get(i).getMeasuredWidth())>(d.getWidth()-left_button.getMeasuredWidth()))
						{
							if (flag_offset_direction==2) flag_offset_direction=3;
							else flag_offset_direction=1;
						}
					}
    				
    				
    				
    			} 
    			while(flag);
    		}
    	}
    	if (FatherLabels!=null)
    	{
    		for (int i=0;i<FatherLabels.size();i++)
    		{	
    			int original_x;
    			int original_y;
    			int flag_offset_direction=0;
    			//per ora si fa cos�, poi sistemeremo!!
    			//FatherLabels.get(i).setWidth(pixels_width_labels);
    			FatherLabels.get(i).measure(d.getWidth(), d.getHeight());
    			int xx=original_x=left_button.getMeasuredWidth()+
    					(int)(Math.random()*(d.getWidth()-2*left_button.getMeasuredWidth()-FatherLabels.get(i).getMeasuredWidth()));
    			int yy=original_y=(int)(Math.random()*(d.getHeight()-FatherLabels.get(i).getMeasuredHeight()));
    			boolean flag;
    			do
    			{
    				flag=false;
    				FatherLabels.get(i).setPosition(xx, yy);
    				//AbsoluteLayout.LayoutParams p=Labels.get(i).randomPose(700,400);
    				FatherLabels.get(i).autoSetParams();
    				//FatherLabels.get(i).autoSetParams(pixels_width_labels);
    				my_layout.addView(FatherLabels.get(i));
    				Log.i("MARCO",FatherLabels.get(i).getPosX()+" "+FatherLabels.get(i).getPosY());
    				//collision detection:
    				Rect r1;
    				Rect r2;
    				FatherLabels.get(i).measure(d.getWidth(), d.getHeight());
					int x1=((AbsoluteLayout.LayoutParams)FatherLabels.get(i).getLayoutParams()).x;
					int y1=((AbsoluteLayout.LayoutParams)FatherLabels.get(i).getLayoutParams()).y;
					r1= new Rect(x1,y1,(x1+FatherLabels.get(i).getMeasuredWidth()),(y1+FatherLabels.get(i).getMeasuredHeight()));
    				for (int h=0;h<i;h++)
    				{
    					FatherLabels.get(h).measure(d.getWidth(), d.getHeight());
    					int x2=((AbsoluteLayout.LayoutParams)FatherLabels.get(h).getLayoutParams()).x;
    					int y2=((AbsoluteLayout.LayoutParams)FatherLabels.get(h).getLayoutParams()).y;
    					r2= new Rect(x2,y2,(x2+FatherLabels.get(h).getMeasuredWidth()),(y2+FatherLabels.get(h).getMeasuredHeight()));
    					if (Rect.intersects(r1, r2)) 
						{
							flag= true;
							Log.i("Info","rectangles "+i+" and "+h+" intersecate!");
						}
    				}
    				
    				for (int h=0;h<SimpleLabels.size();h++)
    				{
    					SimpleLabels.get(h).measure(d.getWidth(), d.getHeight());
    					int x2=((AbsoluteLayout.LayoutParams)SimpleLabels.get(h).getLayoutParams()).x;
    					int y2=((AbsoluteLayout.LayoutParams)SimpleLabels.get(h).getLayoutParams()).y;
    					r2= new Rect(x2,y2,(x2+SimpleLabels.get(h).getMeasuredWidth()),(y2+SimpleLabels.get(h).getMeasuredHeight()));
    					if (Rect.intersects(r1, r2)) 
						{
							flag= true;
							Log.i("Info","rectangles "+i+" and "+h+" intersecate!");
						}
    				}
    				
    				if (flag) 
					{
						my_layout.removeView(FatherLabels.get(i));
						//act.toastMessage("evviva");
						if ((flag_offset_direction==0)||(flag_offset_direction==2)) xx+=COLLISION_OFFSET_STEP;
						else  xx-=COLLISION_OFFSET_STEP;
						if ((flag_offset_direction==0)||(flag_offset_direction==1)) yy+=COLLISION_OFFSET_STEP;
						else yy-=COLLISION_OFFSET_STEP;
						if ((yy+FatherLabels.get(i).getMeasuredHeight())>d.getHeight())
						{
							if (flag_offset_direction==0) flag_offset_direction=2;
							else flag_offset_direction=3;
						}
						if ((xx+FatherLabels.get(i).getMeasuredWidth())>(d.getWidth()-left_button.getMeasuredWidth()))
						{
							if (flag_offset_direction==2) flag_offset_direction=3;
							else flag_offset_direction=1;
						}
					}
    				
    				
    				
    			} 
    			while(flag);
    		}
    	}
    	//add a fake SeekLabel fro debug:
    	/*SeekLabel sl= new SeekLabel(getApplicationContext(),"Curtains",0,100);
    	sl.randomPose(bigL.getWidth(),bigL.getHeight());
    	Log.e("Layout big", "Width: "+Integer.toString(bigL.getWidth()));
    	AbsoluteLayout.LayoutParams p=new AbsoluteLayout.LayoutParams(100, 40, sl.getX(), sl.getY());
    	sl.setLayoutParams(p);
    	bigL.addView(sl);*/
    }
	
	protected void removeAllLabels()
    {
    	for (int i=0;i<SimpleLabels.size();i++)
    	{
    		my_layout.removeView(SimpleLabels.get(i));
    	}
    	for (int i=0;i<FatherLabels.size();i++)
    	{
    		my_layout.removeView(FatherLabels.get(i));
    	}
    	for (int i=0;i<SonLabels.size();i++)
    	{
    		my_layout.removeView(SonLabels.get(i));
    	}
    }
	
	public void refreshImage(Bitmap b)
	{
		image.setImageBitmap(b);
	}
	
	public void refreshImage2(Bitmap b, Matrix m)
	{
		image.setImageBitmap(b);
		image.setImageMatrix(m);
		my_matrix=m;
		image.invalidate();
	}
	
	public void removeSonLabel(RobotSonLabel son)
    {
    	SonLabels.remove(son);
    }
	
	public void addSonLabel(RobotSonLabel son)
    {
    	SonLabels.add(son);
    }
	
    public double getCameraPosition()
    {
    	return cam_position;
    }
    
    public void setCameraPosition(double c)
    {
    	cam_position=c;
    }
    
    public double getTorsoPosition()
    {
    	return this.torso_position;
    }
    
    public void setTorsoPosition(double tp)
    {
    	this.torso_position=tp;
    }
	
	//to send silly messages
	//Toast to send a short message on  the screen
	public Toast toastMessage(String msg)
	{
		CharSequence connessione = msg;
		int duration = Toast.LENGTH_SHORT;
		Toast toast = Toast.makeText(getApplicationContext(), connessione, duration);
		toast.setGravity(Gravity.LEFT | Gravity.LEFT, 5, 5);
		toast.show();
		return toast; 
		    
	}
	
	//to switch to the user view we need to launch the correct activity
	protected void switchToUserView()
	{
		myApp.stopSubscribing();
		final Intent intent = new Intent().setClass(RobotView.this.me, UserView.class);
		RobotView.this.startActivity(intent);
		finish();
	}
	
	//Switching to the "Robot executing command" view, (called when a Button,i.e. command, is pressed)
	public void showRobotExecutingCommandView(String phrase)
	{
		finish();
		myApp.robotBusy();
		final Intent intent = new Intent().setClass(RobotView.this.me, RobotWorkingView.class);
		RobotView.this.startActivity(intent);
	}
	
	 /****************************/
    /* the options menu methods */
    /****************************/
    @Override
    public boolean onCreateOptionsMenu(final android.view.Menu menu) {
		final MenuInflater inflater = this.getMenuInflater();
		inflater.inflate(R.menu.optmenu, menu);
		return (super.onCreateOptionsMenu(menu));
    }
    
    @Override
    public boolean onOptionsItemSelected(final MenuItem item) {
		switch (item.getItemId()) {
		case R.id.close: {
			LayoutInflater layoutInflater 
		     = (LayoutInflater)me.getBaseContext()
		      .getSystemService(me.LAYOUT_INFLATER_SERVICE);  
		    LinearLayout popupView =(LinearLayout)layoutInflater.inflate(R.layout.mypopupclose, null);	    
       	popupView.setOrientation(popupView.VERTICAL);
       	TextView ptv= new TextView(getApplicationContext());
       	ptv.setBackgroundColor(Color.TRANSPARENT);
       	ptv.setTextColor(Color.WHITE);
       	ptv.setText("Close Activity");
       	ptv.setTextSize(22);
       	ptv.setPadding(10, 2, 2, 2);
       	popupView.addView(ptv);
       	Button line= new Button(getApplicationContext());
   		line.setBackgroundColor(Color.WHITE);
   		line.setClickable(false);
   		line.setHeight(2);
   		LinearLayout.LayoutParams p= new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT,
   				LinearLayout.LayoutParams.WRAP_CONTENT);
   		line.setLayoutParams(p);
   		popupView.addView(line);
   		TextView ptv2= new TextView(getApplicationContext());
       	ptv2.setBackgroundColor(Color.TRANSPARENT);
       	ptv2.setTextColor(Color.WHITE);
       	ptv2.setText("Are you sure?");
       	ptv2.setTextSize(18);
       	ptv2.setPadding(10, 5, 0, 5);
       	popupView.addView(ptv2);
   		LinearLayout ll= new LinearLayout(getApplicationContext());
   		ll.setBackgroundColor(Color.TRANSPARENT);
   		ll.setOrientation(ll.HORIZONTAL);
   		ll.setPadding(5, 5, 5, 5);
   		ll.setLayoutParams(new LayoutParams(LayoutParams.WRAP_CONTENT,LayoutParams.WRAP_CONTENT));
       	popupView.addView(ll);
       	Button yes= new Button(getApplicationContext());
       	yes.setText("yes");
       	yes.setWidth(125);
       	yes.setOnClickListener(new View.OnClickListener() {
				
				@Override
				public void onClick(View v) {
					popupWindow.dismiss();
					myApp.closeApp();
				}
			});
       	yes.setLayoutParams(new LayoutParams(LayoutParams.WRAP_CONTENT,
       			LayoutParams.WRAP_CONTENT));
       	ll.addView(yes);
       	
       	Button no= new Button(getApplicationContext());
       	no.setText("no");
       	no.setWidth(125);
       	no.setOnClickListener(new View.OnClickListener() {
				
				@Override
				public void onClick(View v) {
					popupWindow.dismiss();
				}
			});
       	no.setLayoutParams(new LayoutParams(LayoutParams.WRAP_CONTENT,
       			LayoutParams.WRAP_CONTENT));
       	ll.addView(no);
   		
       	popupWindow = new PopupWindow(popupView, 
		               LayoutParams.WRAP_CONTENT,  
		                     LayoutParams.WRAP_CONTENT);
       	popupWindow.setOutsideTouchable(true);
       	popupWindow.setBackgroundDrawable(getResources().getDrawable(R.drawable.transparent));
       	popupWindow.showAtLocation(getWindow().getDecorView(),Gravity.CENTER,0,0);
		}
		return true;
		case R.id.actions_list: {
			myApp.stopSubscribing();
			final Intent intent = new Intent().setClass(RobotView.this.me, ActionsListView.class);
			RobotView.this.startActivity(intent);
			finish();
		}
		return true;
		case R.id.settings: {
			Intent settingsIntent = new Intent(RobotView.this,
    				Settings.class);
    		RobotView.this.startActivityForResult(settingsIntent,SETTINGS_CODE);
		}
		return true;
		default: return super.onOptionsItemSelected(item);
		}
    }
    
    public int getDisplayHeight()
    {
    	Display display = getWindowManager().getDefaultDisplay();
    	return display.getHeight();
    }
    
    public int getDisplayWidth()
    {
    	Display display = getWindowManager().getDefaultDisplay();
    	return display.getWidth();
    }
    
   //Method to manage the callback from a start activity for results (i.e. it runs when you change the settings)
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
    	super.onActivityResult(requestCode, resultCode, data);
    	switch(requestCode){
        	case(SETTINGS_CODE):{
        		if(resultCode==Activity.RESULT_OK)
        		{
        			myPreferences.loadPreferences();
        			myApp.setImagesRate(myPreferences.getImagesRate());
        			//myApp.restartThreads(myPreferences);
        			Log.i("AccompanyGUI","onActivityResult");
        		}
        	} break;
    	}
    }
    
    /****************************/
    
	//to close this activity
	public void halt()
	{
		this.finish();
	}

	/*@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {
		NodeConfiguration nodeConfiguration;
		try {
			turn_talker= new CmdVelocityPublisher();
			head_controller = new HeadControllerGUI();
			nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),
			    new URI("http://192.168.1.109:11311/"));
			nodeMainExecutor.execute(image, nodeConfiguration.setNodeName("AccompanyGUI/RobotView_image"));
			//nodeConfiguration = NodeConfiguration.newPrivate(new URI("http://192.168.1.109:11311/"));
			nodeMainExecutor.execute(turn_talker, nodeConfiguration.setNodeName("AccompanyGUI/CmdVelPub"));
			nodeMainExecutor.execute(head_controller, nodeConfiguration.setNodeName("AccompanyGUI/HeadControllerGUI"));
		} catch (URISyntaxException e) {
			Log.e("Accompany","error in linking imageview mand/or talkers");
		}
		    
	}*/
	
	 @Override
	 public void onDestroy()
	 {
		// Log.i("dim",image.getWidth()+" "+image.getHeight());
		 super.onDestroy();
	 }
	

}

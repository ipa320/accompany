package it.unisi.accompany.activities;

import it.unisi.accompany.AccompanyGUIApp;
import it.unisi.accompany.AccompanyPreferences;
import it.unisi.accompany.widget.ActionPossibilityWidget;
import it.unisi.accompany.R;

import java.io.IOException;
import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import com.hoho.android.usbserial.driver.UsbSerialProber;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.os.Bundle;
import android.os.Handler;
import android.os.StrictMode;
import android.util.Log;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewTreeObserver;
import android.view.ViewTreeObserver.OnPreDrawListener;
import android.widget.AbsoluteLayout;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.PopupWindow;
import android.widget.TextView;
import android.widget.FrameLayout.LayoutParams;

public class UserView extends AccompanyActivity{
	
	protected UserView me;
	
	protected final String TAG = "AccompanyGUI-userView";
	
	protected final int MAXUSERACTIONS=5;
	
	//protected ImageButton left_button;
	//protected ImageButton right_button;
	protected Button switch_to_robot_button;
	@SuppressWarnings("deprecation")
	protected AbsoluteLayout my_layout;
	
	//My Buttons (Possible commands!!)
	protected ArrayList<ActionPossibilityWidget> ActionPossibilities = null;
	
	protected PopupWindow popupWindow;
	protected ImageButton opt_menu;
	
	protected FrameLayout main_layout;
	protected PopupWindow menu;
	
	protected ViewTreeObserver vto;
	protected OnPreDrawListener pdl;
	
	protected int circle_size=0;
	protected int circle_x=0;
	protected int circle_y=0;
	
	//on create
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		//standrd things
		super.onCreate(savedInstanceState);
		this.setContentView(R.layout.user_view);
		Log.i(TAG,"on create userView");
			
		//Setting up the policies for the use of threads
	    if (android.os.Build.VERSION.SDK_INT > 9) {
			StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder()
					.permitAll().build();
			StrictMode.setThreadPolicy(policy);
		}
	        
		//setting the activity variable
		myApp.setUserView(this);
		me=this;
			
		//recovering all the layout stuffs
		my_layout=(AbsoluteLayout)this.findViewById(R.id.user_layout);
		switch_to_robot_button=(Button)this.findViewById(R.id.switchtorobot);
		main_layout=(FrameLayout)this.findViewById(R.id.global_user_layout);
			
		//Setting the listners for the button:
		switch_to_robot_button.setOnClickListener(new View.OnClickListener() {			
		@Override
			public void onClick(View v) {
				switchToRobotView();
			}		});	
			
		//setting the background of the layout clickable (to reset the parent buttons)
		//i.e. if you click on background and there are son labels shown they will be removed
		my_layout.setOnClickListener(new View.OnClickListener() {			
			@Override
			public void onClick(View v) {
				resetAllActionsPossibilities();
			}		});	
			
			
		opt_menu=(ImageButton)this.findViewById(R.id.optmenu);
		opt_menu.setOnClickListener(new View.OnClickListener() {
				
			@Override
			public void onClick(View arg0) {
				showMyMenu();
			}
		});
			
			
		//Thread that waits the db client to be active and send the request for the labels.
		//after a certain amount of time it exits and gives an error message
		final Handler h= new Handler();
		final Thread labelsLoader = new Thread() {
			@Override
			public void run()
			{
				final int max_time=5000;  //5 secondi di attesa del servizo
				int waited=0;
				try{
					while ((!isClientDbWorking())&&(waited < max_time))
					{
						Thread.sleep(100);
						waited+=100;
					}
				} catch(final Exception e){
					
			}
			finally{  //if service is active send request to db
					if (myApp.db_client==null) toastMessage("Error: cannot connect to the Database service!");
					else	//if (myApp.db_client.isStarted()) 
						sendRequest();
				}
			}
		};
		labelsLoader.start();
			
		myApp.setRunningActivity(myApp.USER_VIEW);
			
		vto=this.my_layout.getViewTreeObserver();
		pdl=new OnPreDrawListener(){
			@Override
			public boolean onPreDraw() {
					
				int scarto=135;
					
				ImageView iv= new ImageView(UserView.this);
				iv.setBackgroundColor(Color.parseColor("#00000000"));
				AbsoluteLayout.LayoutParams par;
				int ww= my_layout.getMeasuredWidth()-scarto*2;
				int hh= my_layout.getMeasuredHeight()-scarto*2;
				int size=ww;
				if (size>hh) size=hh;
				par= new AbsoluteLayout.LayoutParams(size,size, 75+scarto, scarto);
				circle_x=75+(int)(scarto+(size/2));
				circle_y=(int)(scarto+(size/2));
				circle_size=size;
				iv.setLayoutParams(par);
				my_layout.addView(iv);		
				Bitmap b = Bitmap.createBitmap( size, size, Bitmap.Config.ARGB_8888);
				Canvas c=new Canvas(b);
				c.drawColor(Color.parseColor("#00000000"));
				RectF oval= new RectF();
				oval.set(10,10,size-10,size-10);
				Paint p= new Paint();
				p.setStyle(Paint.Style.STROKE);
				p.setStrokeWidth(5);
				p.setAntiAlias(true);
				p.setColor(Color.parseColor("#FF989898"));
				c.drawArc(oval,60,210,false,p); //240 x pari
				p= new Paint(Paint.ANTI_ALIAS_FLAG);
				p.setColor(Color.parseColor("#FFD6D6D6"));
				int x= ((int)(size/2))+(int)((size/2-10)*Math.cos(Math.toRadians(60)));
				int y= ((int)(size/2))+(int)((size/2-10)*Math.sin(Math.toRadians(60)));
				c.drawCircle(x, y, 10, p);
				x= ((int)(size/2))+(int)((size/2-10)*Math.cos(Math.toRadians(270)));
				y= ((int)(size/2))+(int)((size/2-10)*Math.sin(Math.toRadians(270)));
				c.drawCircle(x, y, 10, p);
				iv.setImageBitmap(b);
				removeAllLabelsFromScreen();
				if (ActionPossibilities!=null)
					if (ActionPossibilities.isEmpty()) showLabels();
					
				UserView.this.vto=UserView.this.my_layout.getViewTreeObserver();
				vto.removeOnPreDrawListener(UserView.this.pdl);
				return true;
			}
		};
		vto.addOnPreDrawListener(pdl);
	}
	
	@Override
	public void onRestart()
	{
		Log.i(TAG,"on restart userView");
		myPreferences.loadPreferences();
		super.onRestart();
		//removing old labels
		removeAllLabels();
		if (myApp.db_client==null) toastMessage("Error: cannot connect to the Database service!");
				else  sendRequest();
		//SETTING THIS AS THE ACTIVITY FOCUSED
		myApp.setRunningActivity(myApp.USER_VIEW);
	}
	
	@Override
	public void onStart()
	{
		Log.i(TAG,"on start userView");
		super.onStart();
	}
	
	@Override
	public void onResume()
	{
		Log.i("AccompanyGUI","on resume userView");
		super.onResume();
		if (myApp.mSerialDevice==null)
		{
			myApp.mSerialDevice = UsbSerialProber.acquire(myApp.mUsbManager);
	        Log.d("AccompanyGUI-squeeze", "Resumed, mSerialDevice=" + myApp.mSerialDevice);
	        if (myApp.mSerialDevice == null) {
	            Log.i("AccompanyGUI-squeeze","No serial device.");
	        } else {
	            try {
	                myApp.mSerialDevice.open();
	                myApp.mSerialDevice.setBaudRate(9600);
	            } catch (IOException e) {
	                Log.e("AccompanyGUI-squeeze", "Error setting up device: " + e.getMessage(), e);
	                Log.e("AccompanyGUI-squeeze","Error opening device: " + e.getMessage());
	                try {
	                    myApp.mSerialDevice.close();
	                } catch (IOException e2) {
	                    // Ignore.
	                }
	                myApp.mSerialDevice = null;
	                return;
	            }
	            Log.i("AccompanyGUI-squeeze","Serial device: " + myApp.mSerialDevice);
	        }
	        myApp.onDeviceStateChange();
		}
	}
	
	protected boolean isClientDbWorking()
	{
		if (myApp.db_client==null) return false;
		//else if (myApp.db_client.isStarted()) return true;
			else return true;
	}
	
	public void handleResponse(String res)
	{

    	if (ActionPossibilities==null) ActionPossibilities= new ArrayList<ActionPossibilityWidget>();
    	else ActionPossibilities.clear();
    	
    	Log.e(TAG,"USER RESPONSE: "+res);
    	
    	//parse json data
        try{
            JSONArray jArray = new JSONArray(res);
            for(int i=0;i<jArray.length();i++){
                JSONObject json_data = jArray.getJSONObject(i);
                ActionPossibilities.add(new ActionPossibilityWidget(getApplicationContext(),
                	json_data.getInt("apId"),
                	json_data.getString("ap_label"),json_data.getDouble("likelihood"),
                	json_data.getString("phraseal_feedback"),
                	my_layout,this,
                	json_data.getInt("precond_id"),true));
            }
        }catch(JSONException e){
            Log.e(TAG, " user response: Error parsing data "+e.toString());
        }
        
        filterUserActions();
        
        showLabels();
	}
	
	@Override
	public void showLabels() 
	{
		if (ActionPossibilities.size()>2)
		{  //makes the swap (for Michele)
			ActionPossibilityWidget apw = ActionPossibilities.remove(0);
			ActionPossibilities.add(1, apw);
		}
		
		int x,y;
		double butt_radius,next_r;
		int margine=5;
		double raggio=circle_size/2-10;//prima senza -10
		double angolo=Math.PI/2;
		x= circle_x+(int)(raggio*Math.cos(angolo));
		y= circle_y+(int)(raggio*Math.sin(angolo));
		AbsoluteLayout.LayoutParams par;
		for (int i=0;i<ActionPossibilities.size();i++)
		{
			int x1,y1;
			x1=x-((int)(ActionPossibilities.get(i).getMyWidth()/2));
			y1=y-((int)(ActionPossibilities.get(i).getMyWidth()/2));
			//par= new AbsoluteLayout.LayoutParams(AbsoluteLayout.LayoutParams.WRAP_CONTENT, AbsoluteLayout.LayoutParams.WRAP_CONTENT, x1, y1);
			//ActionPossibilities.get(i).setLayoutParams(par);
			//my_layout.addView(ActionPossibilities.get(i));
			ActionPossibilities.get(i).addToLayout(x1,y1);
			Log.i(TAG,"Adding ap to layout: "+ActionPossibilities.get(i).getName());
			butt_radius=ActionPossibilities.get(i).getMyWidth()/2;
			if (i==ActionPossibilities.size()-1)
				next_r=ActionPossibilities.get(0).getMyWidth()/2;
			else
				next_r=ActionPossibilities.get(i+1).getMyWidth()/2;
			angolo=angolo+(2*Math.asin((butt_radius + margine+ next_r)/(2*raggio)));
			x= circle_x+(int)(raggio*Math.cos(angolo));
			y= circle_y+(int)(raggio*Math.sin(angolo));
		}
	}
	
	@Override
	public void removeAllLabels() {
		if (ActionPossibilities!=null)
		{
			for (int i=0;i<ActionPossibilities.size();i++)
			{
				ActionPossibilities.get(i).removeFromLayout();
			}
			ActionPossibilities.clear();
		}
	}
	
	public void removeAllLabelsFromScreen()
	{
		if (ActionPossibilities!=null)
		{
			for (int i=0;i<ActionPossibilities.size();i++)
			{
				ActionPossibilities.get(i).removeFromLayout();
			}
		}
	}
	
	@Override
	public void filterUserActions() {
		Log.i(TAG,"Before filtering, Buttons array dimensions are:  "+ ActionPossibilities.size());
    	while(ActionPossibilities.size()>MAXUSERACTIONS)
    	{
    		ActionPossibilities.remove(ActionPossibilities.size()-1);
    	}
	}
	
	@Override
	public void resetAllActionsPossibilities() {
		if(ActionPossibilities!=null)
			for (int i=0;i< ActionPossibilities.size();i++)
				ActionPossibilities.get(i).resetMe();
	}
	
	@Override
	public void resetAllActionsPossibilities(ActionPossibilityWidget apw) {
		if(ActionPossibilities!=null)
			for (int i=0;i< ActionPossibilities.size();i++)
				if (!ActionPossibilities.get(i).equals(apw)) ActionPossibilities.get(i).resetMe();
	}

	@Override
	public void sendRequest() {
		myApp.RequestToDB(myApp.USER_ACTIONS_REQUEST_CODE);
	}

	@Override
	public void sendActionRequest(int id) {
			myApp.sendActionRequest(id);
	}
	
	@Override
	public void commandRunning(boolean b)
	{
		if (b)
		{
			waitRunning();
			//for (int h=0; h<ActionPossibilities.size();h++)
			//	ActionPossibilities.get(h).setClickable(false);
		}
		else
		{
			my_layout.setClickable(true);
			if (ActionPossibilities!=null)for (int h=0; h<ActionPossibilities.size();h++)
				ActionPossibilities.get(h).setClickable(true);
		}
	}


	//Switching to robot view (on banners clicks), the robot view activity is launched:
	public void switchToRobotView()
	{
		myApp.StartSubscribing();
		final Intent intent = new Intent().setClass(UserView.this.me, RobotView.class);
		UserView.this.startActivity(intent);
		finish();
	}
	
	//Switching to the "Robot executing command" view, (called when a Button,i.e. command, is pressed)
	public void showRobotExecutingCommandView(String phrase)
	{
		Log.i("123","user - RobotShowExecutingCommandView start...");
		myApp.st.setMode(false);
		myApp.StartSubscribing();
		myApp.robotBusy();
		my_layout.setClickable(true);
		final Intent intent = new Intent().setClass(UserView.this.me, RobotWorkingView.class);
		UserView.this.startActivity(intent);
		finish();
	}
	
	public void showRobotExecutingCommandView()
	{
		Log.i("123","user - RobotShowExecutingCommandView start...");
		myApp.st.setMode(false);
		myApp.StartSubscribing();
		myApp.robotBusy();
		final Intent intent = new Intent().setClass(UserView.this.me, RobotWorkingView.class);
		UserView.this.startActivity(intent);
		finish();
	}
	
	public AccompanyPreferences getPreferences()
	{
	  	return myPreferences;
	}
	
	
	/****************************/
    /* the options menu methods */
    /****************************/
	
	protected void showMyMenu()
    {
    	LayoutInflater layoutInflater 
	     = (LayoutInflater)getBaseContext()
	      .getSystemService(LAYOUT_INFLATER_SERVICE);  
	    LinearLayout popupView =(LinearLayout)layoutInflater.inflate(R.layout.my_opt_menu, null);
	    Button act_btn=(Button)popupView.findViewById(R.id.act_btn);
	    act_btn.setWidth(this.getDisplayWidth()/4);
	    Button sett_btn=(Button)popupView.findViewById(R.id.Sett_btn);
	    sett_btn.setWidth(this.getDisplayWidth()/4);
	    Button voice_btn=(Button)popupView.findViewById(R.id.voice_btn);
	    voice_btn.setWidth(this.getDisplayWidth()/4);
	    if (myPreferences.getSpeechMode())
	    	voice_btn.setText("Voice Off");
	    else
	    	voice_btn.setText("Voice On");
	    Button close_btn=(Button)popupView.findViewById(R.id.Close_btn);
	    close_btn.setWidth(this.getDisplayWidth()/4);
	    menu = new PopupWindow(popupView,LayoutParams.MATCH_PARENT,100,true);
	    menu.setContentView(popupView);
		menu.setOutsideTouchable(true);
       	menu.setBackgroundDrawable(getResources().getDrawable(R.drawable.transparent));
	    Log.e("aa",main_layout.getMeasuredHeight()+ " "+popupView.getHeight());
	    menu.showAtLocation(this.findViewById(R.id.global_user_layout), 0, 0, (main_layout.getMeasuredHeight()-popupView.getHeight()));
	    close_btn.setOnClickListener(new View.OnClickListener() {		
			@Override
			public void onClick(View arg0) {
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
	   				2);
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
	       	yes.setTextColor(Color.WHITE);
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
	       	no.setTextColor(Color.WHITE);
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
	       	
	       	menu.dismiss();
			}
		});
	    sett_btn.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				myApp.setSettings();
				Intent settingsIntent = new Intent(UserView.this,
	    				Settings.class);
	    		UserView.this.startActivityForResult(settingsIntent,SETTINGS_CODE);
	    		menu.dismiss();
			}
		});
	    act_btn.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				final Intent intent = new Intent().setClass(UserView.this.me, ActionsListView.class);
				UserView.this.startActivity(intent);
				finish();
			}
		});
	    voice_btn.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				if (myPreferences.getSpeechMode()) 
				{
					setPreferences(false);
					myApp.stopSpeechrecognition();
				}
				else 
				{
					setPreferences(true);
					myApp.startSpeechRecognition();
				}
				myPreferences.loadPreferences();
				menu.dismiss();
			}
		});
    }

	@Override
	public void isMovingButton(boolean b) {
		my_layout.setClickable(!b);
	}

	@Override
	public void waitRunning() {
		my_layout.setClickable(false);
		if (ActionPossibilities!=null) for (int i=0;i<ActionPossibilities.size();i++)
			ActionPossibilities.get(i).setRunningTask();
	}
	

}

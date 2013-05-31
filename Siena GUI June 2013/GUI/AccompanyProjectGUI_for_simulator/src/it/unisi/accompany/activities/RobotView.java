package it.unisi.accompany.activities;

import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import it.unisi.accompany.threads.MaskAnimationThread;
import it.unisi.accompany.threads.MaskAnimationThreadWorking;
import it.unisi.accompany.widget.ActionPossibilityWidget;
import it.unisi.accompany.R;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.Rect;
import android.os.Bundle;
import android.os.Handler;
import android.os.StrictMode;
import android.util.Log;
import android.view.Display;
import android.view.GestureDetector;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.widget.AbsoluteLayout;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.PopupWindow;
import android.widget.TextView;
import android.widget.FrameLayout.LayoutParams;

public class RobotView extends AccompanyActivity{

	protected final String TAG = "Accompany GUI - Robot View";
	
	protected final int MAXUSERACTIONS=4;
	
	protected RobotView me;

	//Layout asn similar
	protected FrameLayout main_layout;
	protected Button switch_to_user_button;
	protected FrameLayout mainButtonsLayout;
	protected AbsoluteLayout my_layout;
	protected ImageView image;
	public ImageView mask,mask1,mask2,mask3,mask4,mask5,mask6,mask7,maskF;
	
	//robot joints:
	protected double cam_position=-3.1415926;
	protected double torso_position=0;
	
	//Gesture detection
	protected View.OnTouchListener gl;
	
	//popup
	protected PopupWindow popupWindow;
	
	//fling detection
	boolean flag_move;
	int myLastTouch_x;
	int myLastTouch_y;
	int pointerId;
	private final int INVALID_POINTER_ID=-1;
	private final int PIXEL_TRESHOLD_GESTURE=5;
	
	//options menu stuffs
	protected ImageButton opt_menu;
	protected PopupWindow menu;
	
	//the action possibilities 
	protected ArrayList<ActionPossibilityWidget> ActionPossibilities = null;
	
	//the mask animation management
	protected Handler animationHandler;
	protected MaskAnimationThread mt;

	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
		this.setContentView(R.layout.robot_view);
		
		Log.i(TAG,"on create!");
		
		//Setting up the policies for the use of threads
        if (android.os.Build.VERSION.SDK_INT > 9) {
			StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder()
					.permitAll().build();
			StrictMode.setThreadPolicy(policy);
		}
		
		myApp.setRobotView(this);
		me=this;
		//bring camera to front
		myApp.head_controller.bringCameraToFront();
		myApp.torso_controller.bringHome();
		myApp.setRunningActivity(myApp.ROBOT_VIEW);
		
		//getting the layout
		main_layout=(FrameLayout)this.findViewById(R.id.robot_global_layout);
        switch_to_user_button=(Button)this.findViewById(R.id.switchtouser);
        mainButtonsLayout=(FrameLayout)this.findViewById(R.id.robotview_buttons_main_layout);
		my_layout=(AbsoluteLayout)this.findViewById(R.id.robot_layout);
		mask=(ImageView)this.findViewById(R.id.mask_basic_robot);
		mask1=(ImageView)this.findViewById(R.id.mask_basic_1);
		mask2=(ImageView)this.findViewById(R.id.mask_basic_2);
		mask3=(ImageView)this.findViewById(R.id.mask_basic_3);
		mask4=(ImageView)this.findViewById(R.id.mask_basic_4);
		mask5=(ImageView)this.findViewById(R.id.mask_basic_5);
		mask6=(ImageView)this.findViewById(R.id.mask_basic_6);
		mask7=(ImageView)this.findViewById(R.id.mask_basic_7);
		maskF=(ImageView)this.findViewById(R.id.mask_basic_F);
		image=(ImageView)this.findViewById(R.id.robot_standard_image_view);
		image.setImageBitmap(myApp.getLastImage());
		
		//setting buttons listeners
		switch_to_user_button.setOnClickListener(new View.OnClickListener() {			
			@Override
			public void onClick(View v) {
				switchToUserView();
			}		});	
		
		opt_menu=(ImageButton)this.findViewById(R.id.optmenu);
	    opt_menu.setOnClickListener(new View.OnClickListener() {
			
			@Override
			public void onClick(View arg0) {
				showMyMenu();
			}
		});
	    
	    gl = new View.OnTouchListener() {
			@Override
			public boolean onTouch(View v, MotionEvent event) {	
				
				final int action=event.getAction();
				switch (action & MotionEvent.ACTION_MASK)
				{
				case MotionEvent.ACTION_DOWN:
				{
					Log.d(TAG + " (gesture detetcion)","Action down");
					flag_move=false;
					myLastTouch_x=(int)event.getX();
					myLastTouch_y=(int)event.getY();
					pointerId=event.getPointerId(0);
					//remove labels (e banners se possibile)
					my_layout.removeAllViews();
					my_layout.invalidate();
					break;
				}
				case MotionEvent.ACTION_MOVE:
				{
					Log.d(TAG + " (gesture detetcion)","Action move");
					flag_move=true;
					final int act_p=event.getPointerId(pointerId);
					final float x= event.getX();
					final float y= event.getY();
					final float dx= -(x-myLastTouch_x);
					float dy= -(y-myLastTouch_y);
					//move! (send message on topic!!)
					if (Math.abs(dx) > PIXEL_TRESHOLD_GESTURE)
						myApp.turn_talker.publish(-(float)(dx*Math.PI/180)/2);                   //Turn base!
					
					Log.i("head pos",myApp.head_controller.getHeadPos()+"");
					if (Math.abs(dy) > PIXEL_TRESHOLD_GESTURE)
					{
						if (myApp.head_controller.getHeadPos()<-(myApp.head_controller.max_head_pos-myApp.head_controller.min_head_pos)/2)
							dy=-dy;
						double diff=myApp.head_controller.publish((dy*Math.PI/180)/10);   //Turn Head!
						Log.i(TAG + " (gesture detetcion)","Head difference: "+diff+"");
						if (diff!=0) myApp.torso_controller.publish(-diff);
					}
					
					myLastTouch_x=(int)x;
					myLastTouch_y=(int)y;
					break;
				}
				case MotionEvent.ACTION_UP:
				{
					Log.d(TAG + " (gesture detetcion)","Action up");
					// show labels again
					//getRobotLabelsFromDB();
					//addLabels(my_layout);
					if (isClientDbWorking()) sendRequest();
					
					
					pointerId= INVALID_POINTER_ID;
					break;
				}
				case MotionEvent.ACTION_CANCEL:
				{
					Log.d(TAG + " (gesture detetcion)","Action cancel");
					// show labels again
					//getRobotLabelsFromDB();
					//addLabels(my_layout);
					if (isClientDbWorking()) sendRequest();
					
					pointerId= INVALID_POINTER_ID;
					break;
				}
				case MotionEvent.ACTION_POINTER_UP:
				{
					Log.d(TAG + " (gesture detetcion)","Action pointer up");
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
        
        //selecting correct mask
        setMask();
        
        if (isClientDbWorking()) sendRequest();
        
        animationHandler= new Handler();
        mt=new MaskAnimationThread(animationHandler, this);
        mt.start();
		
	}
	
	@Override
	public void onDestroy()
	{
		mt.terminate();
		super.onDestroy();
	}
	
	@Override
	public void onResume()
	{
		super.onResume();
		//mt.continueRun();
	}
	
	@Override
	public void onRestart()
	{
		Log.i("AccompanyGUI","on restart robotView");
		myPreferences.loadPreferences();
		super.onRestart();
		if (isClientDbWorking()) sendRequest();
		//SETTING THIS AS THE ACTIVITY FOCUSED
		myApp.setRunningActivity(myApp.ROBOT_VIEW);
		mt.continueRun();
	}
	
	@Override
	public void sendRequest() {
		myApp.RequestToDB(myApp.ROBOT_ACTIONS_REQUEST_CODE);
	}

	@Override
	public void sendActionRequest(int id) {
		myApp.sendActionRequest(id);
	}
	
	public void handleResponse(String res)
	{
		//remove all labels from layout:
    	//removeAllLabels();
		
		if (ActionPossibilities==null) ActionPossibilities= new ArrayList<ActionPossibilityWidget>();
    	else ActionPossibilities.clear();

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
                	json_data.getInt("precond_id"),false));
            }
        }catch(JSONException e){
            Log.e(TAG, " user response: Error parsing data "+e.toString());
        }
        
        filterUserActions();
        
        addLabels(my_layout);
	}
	
	protected boolean isClientDbWorking()
	{
		if (myApp.db_client==null) return false;
			else return true;
	}
	
	@SuppressWarnings("deprecation")
	public void addLabelsOld(View v)
    {
    	final int COLLISION_OFFSET_STEP=5;
    	
    	Display d=getWindowManager().getDefaultDisplay();

    	if (ActionPossibilities!=null)
    	{
    		for (int i=0;i<ActionPossibilities.size();i++)
    		{	
    			int original_x;
    			int original_y;
    			int flag_offset_direction=0;
    			ActionPossibilities.get(i).measure(d.getWidth(), d.getHeight());
    			int xx=original_x=(int)(Math.random()*(d.getWidth()-ActionPossibilities.get(i).getMeasuredWidth()));
    			int yy=original_y=(int)(Math.random()*(d.getHeight()-ActionPossibilities.get(i).getMeasuredHeight()));
    			boolean flag;
    			do
    			{
    				flag=false;
    				//ActionPossibilities.get(i).setPosition(xx, yy);
    				ActionPossibilities.get(i).addToLayout(xx, yy);
    				Rect r1;
    				Rect r2;
    				ActionPossibilities.get(i).measure(d.getWidth(), d.getHeight());
					int x1=((AbsoluteLayout.LayoutParams)ActionPossibilities.get(i).getLayoutParams()).x;
					int y1=((AbsoluteLayout.LayoutParams)ActionPossibilities.get(i).getLayoutParams()).y;
					r1= new Rect(x1,y1,(x1+ActionPossibilities.get(i).getMeasuredWidth()),(y1+ActionPossibilities.get(i).getMeasuredHeight()));
    				for (int h=0;h<i;h++)
    				{
    					ActionPossibilities.get(h).measure(d.getWidth(), d.getHeight());
    					int x2=((AbsoluteLayout.LayoutParams)ActionPossibilities.get(h).getLayoutParams()).x;
    					int y2=((AbsoluteLayout.LayoutParams)ActionPossibilities.get(h).getLayoutParams()).y;
    					r2= new Rect(x2,y2,(x2+ActionPossibilities.get(h).getMeasuredWidth()),(y2+ActionPossibilities.get(h).getMeasuredHeight()));
    					if (Rect.intersects(r1, r2)) 
						{
							flag= true;
							Log.i("Info","rectangles "+i+" and "+h+" intersecate!");
						}
    				}
    				
    				if (flag) 
					{
						//my_layout.removeView(ActionPossibilities.get(i));
    					ActionPossibilities.get(i).removeFromLayout();
						if ((flag_offset_direction==0)||(flag_offset_direction==2)) xx+=COLLISION_OFFSET_STEP;
						else  xx-=COLLISION_OFFSET_STEP;
						if ((flag_offset_direction==0)||(flag_offset_direction==1)) yy+=COLLISION_OFFSET_STEP;
						else yy-=COLLISION_OFFSET_STEP;
						if ((yy+ActionPossibilities.get(i).getMeasuredHeight())>d.getHeight())
						{
							if (flag_offset_direction==0) flag_offset_direction=2;
							else flag_offset_direction=3;
						}
						if ((xx+ActionPossibilities.get(i).getMeasuredWidth())>(d.getWidth()))
						{
							if (flag_offset_direction==2) flag_offset_direction=3;
							else flag_offset_direction=1;
						}
					}
    			} 
    			while(flag);
    		}
    	}
    }
	
	@SuppressWarnings("deprecation")
	public void addLabels(View v)
    {
    	
    	Display d=getWindowManager().getDefaultDisplay();

    	ArrayList<Integer> x_coords= new ArrayList<Integer>();
		ArrayList<Integer> y_coords= new ArrayList<Integer>();
    	
    	if (ActionPossibilities!=null)
    	{
    		for (int i=0;i<ActionPossibilities.size();i++)
    		{	
    			Log.v(TAG+"-addAPs","display width:"+d.getWidth()+". height: "+d.getHeight());
    			Log.v(TAG+"-addAPs","ap width:"+ActionPossibilities.get(i).getMyWidth());
    			int xxx= ((int)(Math.random()*(d.getWidth()-ActionPossibilities.get(i).getMyWidth())));
    			int yyy= ((int)(Math.random()*(d.getHeight()-ActionPossibilities.get(i).getMyWidth())));
    			Log.v(TAG+"-addAPs","generated: "+ xxx+","+yyy);
    			x_coords.add(xxx);
    			y_coords.add(yyy);
    			boolean flag;
    			do
    			{
    				flag=false;
    				for (int h=0;h<i;h++)
    				{
    					int distance= (int)Math.sqrt((x_coords.get(i)-x_coords.get(h))*(x_coords.get(i)-x_coords.get(h))+
    							(y_coords.get(i)-y_coords.get(h))*(y_coords.get(i)-y_coords.get(h)));
    					if (distance <((ActionPossibilities.get(i).getMyWidth()/2)+(ActionPossibilities.get(h).getMyWidth())))
    						flag=true;
    					
    					if (flag)
    					{
    						Log.v(TAG+"-addAPs","flag!");
    						x_coords.remove(x_coords.size()-1);
    						y_coords.remove(y_coords.size()-1);
    						x_coords.add((int)(Math.random()*(d.getWidth()-ActionPossibilities.get(i).getMyWidth())));
    		    			y_coords.add((int)(Math.random()*(d.getHeight()-ActionPossibilities.get(i).getMyWidth())));
    					}
    				}
    			} 
    			while(flag);
    		}
    		
    		//true layout show
    		for (int i=0;i<ActionPossibilities.size();i++)
    		{
    			Log.v(TAG+"-addAPs","Adding "+ActionPossibilities.get(i).getName()+". In: "+x_coords.get(i)+","+y_coords.get(i));
    			ActionPossibilities.get(i).addToLayout(x_coords.get(i), y_coords.get(i));
    		}
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
    	my_layout.removeAllViews();
    	mainButtonsLayout.removeView(my_layout);
    	my_layout=new AbsoluteLayout(this);
    	my_layout.setBackgroundColor(Color.TRANSPARENT);
    	FrameLayout.LayoutParams paras= new FrameLayout.LayoutParams(FrameLayout.LayoutParams.MATCH_PARENT, FrameLayout.LayoutParams.MATCH_PARENT);
    	my_layout.setLayoutParams(paras);
    	mainButtonsLayout.addView(my_layout);
    	my_layout.setOnTouchListener(gl);
	}

	@Override
	public void showLabels() {
		// unused
	}
	
	public void refreshImage(Bitmap b)
	{
		image.setImageBitmap(b);
	}
	
	//ROBOT JOINTS CONTROL
	
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
	
	/*********************************************
	 *           SWITCHING VIEWS                 *
	 *********************************************/
	
	//to switch to the user view we need to launch the correct activity
	protected void switchToUserView()
	{
		myApp.stopSubscribing();
		mt.pause();
		final Intent intent = new Intent().setClass(RobotView.this.me, UserView.class);
		RobotView.this.startActivity(intent);
		finish();
	}
		
	//Switching to the "Robot executing command" view, (called when a Button,i.e. command, is pressed)
	public void showRobotExecutingCommandView(String phrase)
	{
		Log.i("123","robot - RobotShowExecutingCommandView start...");
		mt.pause();
		myApp.st.setMode(false);
		myApp.robotBusy();
		my_layout.setClickable(true);
		final Intent intent = new Intent().setClass(RobotView.this.me, RobotWorkingView.class);
		RobotView.this.startActivity(intent);
		finish();
	}
		
	public void showRobotExecutingCommandView()
	{
		Log.i("123","robot -RobotShowExecutingCommandView start...");
		myApp.st.setMode(false);
		mt.pause();
		myApp.robotBusy();
		my_layout.setClickable(true);
		final Intent intent = new Intent().setClass(RobotView.this.me, RobotWorkingView.class);
		RobotView.this.startActivity(intent);
		finish();
	}

	
	@Override
	public void halt()
	{
		mt.terminate();
		super.halt();
	}
	
	//option menu
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
	    menu.showAtLocation(this.findViewById(R.id.robot_global_layout), 0, 0, (main_layout.getMeasuredHeight()-popupView.getHeight()));
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
	       	yes.setWidth(125);
	       	//yes.setBackgroundColor(Color.BLACK);
	       	yes.setTextColor(Color.WHITE);
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
				Intent settingsIntent = new Intent(RobotView.this,
	    				Settings.class);
	    		RobotView.this.startActivityForResult(settingsIntent,SETTINGS_CODE);
	    		menu.dismiss();
			}
		});
	    act_btn.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				myApp.stopSubscribing();
				final Intent intent = new Intent().setClass(RobotView.this.me, ActionsListView.class);
				RobotView.this.startActivity(intent);
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
	
	protected void setMask()
	{
		int ans=myApp.getCurrentExpression();
		switch(ans)
		{
			case 0:
				break; //la normal è già settata di default
			case 1:
				mask1.setBackgroundResource(R.drawable.basic_sadness_one);
				mask2.setBackgroundResource(R.drawable.basic_sadness_two);
				mask3.setBackgroundResource(R.drawable.basic_sadness_three);
				mask4.setBackgroundResource(R.drawable.basic_sadness_four);
				mask5.setBackgroundResource(R.drawable.basic_sadness_five);
				mask6.setBackgroundResource(R.drawable.basic_sadness_six);
				mask7.setBackgroundResource(R.drawable.basic_sadness_seven);
				maskF.setBackgroundResource(R.drawable.mask_sadness);
				mask.setVisibility(View.INVISIBLE);
				maskF.setVisibility(View.VISIBLE);
				break;
			case 2:
				mask1.setBackgroundResource(R.drawable.basic_fear_one);
				mask2.setBackgroundResource(R.drawable.basic_fear_two);
				mask3.setBackgroundResource(R.drawable.basic_fear_three);
				mask4.setBackgroundResource(R.drawable.basic_fear_four);
				mask5.setBackgroundResource(R.drawable.basic_fear_five);
				mask6.setBackgroundResource(R.drawable.basic_fear_six);
				mask7.setBackgroundResource(R.drawable.basic_fear_seven);
				maskF.setBackgroundResource(R.drawable.mask_fear);
				mask.setVisibility(View.INVISIBLE);
				maskF.setVisibility(View.VISIBLE);
				break;
			case 3:
				mask1.setBackgroundResource(R.drawable.basic_disgust_one);
				mask2.setBackgroundResource(R.drawable.basic_disgust_two);
				mask3.setBackgroundResource(R.drawable.basic_disgust_three);
				mask4.setBackgroundResource(R.drawable.basic_disgust_four);
				mask5.setBackgroundResource(R.drawable.basic_disgust_five);
				mask6.setBackgroundResource(R.drawable.basic_disgust_six);
				mask7.setBackgroundResource(R.drawable.basic_disgust_seven);
				maskF.setBackgroundResource(R.drawable.mask_disgust);
				mask.setVisibility(View.INVISIBLE);
				maskF.setVisibility(View.VISIBLE);
				break;
			default:
				break;
		}
	}
	
	 public void switchMask(int b, int a)
	    {
	    	
	    	if (a==b) return;
	    	if (b==0) //se siamo in basic
	    	{
	    		switch(a)
	    		{
	    			case 1:
	    				mask1.setBackgroundResource(R.drawable.basic_sadness_one);
	    				mask2.setBackgroundResource(R.drawable.basic_sadness_two);
	    				mask3.setBackgroundResource(R.drawable.basic_sadness_three);
	    				mask4.setBackgroundResource(R.drawable.basic_sadness_four);
	    				mask5.setBackgroundResource(R.drawable.basic_sadness_five);
	    				mask6.setBackgroundResource(R.drawable.basic_sadness_six);
	    				mask7.setBackgroundResource(R.drawable.basic_sadness_seven);
	    				maskF.setBackgroundResource(R.drawable.mask_sadness);
	    				break;
	    			case 2:
	    				mask1.setBackgroundResource(R.drawable.basic_fear_one);
	    				mask2.setBackgroundResource(R.drawable.basic_fear_two);
	    				mask3.setBackgroundResource(R.drawable.basic_fear_three);
	    				mask4.setBackgroundResource(R.drawable.basic_fear_four);
	    				mask5.setBackgroundResource(R.drawable.basic_fear_five);
	    				mask6.setBackgroundResource(R.drawable.basic_fear_six);
	    				mask7.setBackgroundResource(R.drawable.basic_fear_seven);
	    				maskF.setBackgroundResource(R.drawable.mask_fear);
	    				break;
	    			case 3:
	    				mask1.setBackgroundResource(R.drawable.basic_disgust_one);
	    				mask2.setBackgroundResource(R.drawable.basic_disgust_two);
	    				mask3.setBackgroundResource(R.drawable.basic_disgust_three);
	    				mask4.setBackgroundResource(R.drawable.basic_disgust_four);
	    				mask5.setBackgroundResource(R.drawable.basic_disgust_five);
	    				mask6.setBackgroundResource(R.drawable.basic_disgust_six);
	    				mask7.setBackgroundResource(R.drawable.basic_disgust_seven);
	    				maskF.setBackgroundResource(R.drawable.mask_disgust);
	    				break;
	    			default: break;
	    		}
	    		mt.shot2(false);
	    	}
	    	else //altrimenti
	    	{
	    		mt.shot2(true);
	    	}
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
			if (ActionPossibilities!=null) for (int h=0; h<ActionPossibilities.size();h++)
				ActionPossibilities.get(h).setClickable(true);
		}
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

package it.unisi.accompany.activities;

import it.unisi.accompany.AccompanyGUIApp;
import it.unisi.accompany.AccompanyPreferences;
import it.unisi.accompany.threads.MaskAnimationThreadWorking;
import it.unisi.accompany.R;
import android.app.Activity;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.drawable.AnimationDrawable;
import android.os.Bundle;
import android.os.Handler;
import android.os.StrictMode;
import android.util.Log;
import android.view.Display;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.animation.AnimationUtils;
import android.widget.AbsoluteLayout;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.PopupWindow;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.FrameLayout.LayoutParams;

public class RobotWorkingView extends Activity {
	
	protected final String TAG = "Accomapny GUI - working view";
	
	protected AccompanyGUIApp myApp;
	protected RobotWorkingView me;
	
	protected AbsoluteLayout my_layout;
	public ImageView image;
	public ImageView mask,mask1,mask2,mask3,mask4,mask5,mask6,mask7,maskF;
	
	protected PopupWindow popupWindow;
	protected AccompanyPreferences myPreferences;
	
	protected ImageButton opt_menu;
	
	protected FrameLayout main_layout;
	protected PopupWindow menu;
	protected Button runningAction;
	
	protected Matrix my_matrix;
	
	protected final int MYRUNNINGWIDTH=100;
	protected final int MYRUNNINGTEXTSIZE=10;
	protected final int running_X=1100;
	protected final int running_Y=650;
	
	protected Handler animationHandler;
	protected MaskAnimationThreadWorking mt;
	
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		//standrd things
		super.onCreate(savedInstanceState);
		this.setContentView(R.layout.robot_working_view);
		
		//recovering the application that owns this activity
		//and setting the activity variable
		myApp=(AccompanyGUIApp)this.getApplication();
		myApp.setRobotWorkingView(this);
		me=this;
		
		//bring camera to front
		myApp.head_controller.bringCameraToFront();
		myApp.torso_controller.bringHome();
		
		//reading the preferences 
		myPreferences=new AccompanyPreferences(this);
		myPreferences.loadPreferences();
		
		 //Setting up the policies for the use of threads
        if (android.os.Build.VERSION.SDK_INT > 9) {
			StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder()
					.permitAll().build();
			StrictMode.setThreadPolicy(policy);
		}
		
		image=(ImageView)this.findViewById(R.id.robot_working_image_view);
		try{
			image.setImageBitmap(myApp.getLastImage());
		}catch(Exception e)
		{
			Log.w(TAG,"Attention: null robot image when going in working view!");
		}
		
		mask=(ImageView)this.findViewById(R.id.mask_basic_working);
		mask1=(ImageView)this.findViewById(R.id.mask_basic_working1);
		mask2=(ImageView)this.findViewById(R.id.mask_basic_working2);
		mask3=(ImageView)this.findViewById(R.id.mask_basic_working3);
		mask4=(ImageView)this.findViewById(R.id.mask_basic_working4);
		mask5=(ImageView)this.findViewById(R.id.mask_basic_working5);
		mask6=(ImageView)this.findViewById(R.id.mask_basic_working6);
		mask7=(ImageView)this.findViewById(R.id.mask_basic_working7);
		maskF=(ImageView)this.findViewById(R.id.mask_basic_workingF);
		
		opt_menu=(ImageButton)this.findViewById(R.id.optmenu);
	    opt_menu.setOnClickListener(new View.OnClickListener() {
			
			@Override
			public void onClick(View arg0) {
				showMyMenu();
			}
		});
	    
	    main_layout=(FrameLayout)this.findViewById(R.id.robot_working_global_layout);
		
		Log.i("AccompanyGUI","on create robotWorkingView");
		myApp.setRunningActivity(myApp.EXECUTE_VIEW);
		
		//select the correct inital mask
		setMask();
		runningAction=(Button)this.findViewById(R.id.current_working_action);
		runningAction.setText(myApp.getCurrentTask());	
		runningAction.startAnimation(AnimationUtils.loadAnimation(this, R.anim.rotation));
	//	main_layout.addView(runningAction);
		
		animationHandler= new Handler();
        mt=new MaskAnimationThreadWorking(animationHandler, this);
        mt.start();
	}
	
	@Override
	public void onRestart()
	{
		super.onRestart();
		mt.continueRun();
	}
	
	@Override
	public void onResume()
	{
		super.onResume();
		//mt.continueRun();
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
	public void switchToUserView()
	{
		mt.pause();
		myApp.stopSubscribing();
		final Intent intent = new Intent().setClass(RobotWorkingView.this.me, UserView.class);
		RobotWorkingView.this.startActivity(intent);
		finish();
	}
	
	//Switching to robot view (on banners clicks), the robot view activity is launched:
	public void switchToRobotView()
	{
		mt.pause();
		final Intent intent = new Intent().setClass(RobotWorkingView.this.me, RobotView.class);
		RobotWorkingView.this.startActivity(intent);
		finish();
	}
	
	//Switching to robot view (on banners clicks), the robot view activity is launched:
	public void switchToActionsList()
	{
		mt.pause();
		myApp.stopSubscribing();
		final Intent intent = new Intent().setClass(RobotWorkingView.this.me, ActionsListView.class);
		RobotWorkingView.this.startActivity(intent);
		finish();
	}
	
	@Override
	public void onDestroy()
	{
		mt.terminate();
		super.onDestroy();
	}
	
	 /****************************/
    /* the options menu methods */
    /****************************/
    @Override
    public boolean onCreateOptionsMenu(final android.view.Menu menu) {
    	return super.onCreateOptionsMenu(menu);
    }
    
    @Override
    public boolean onOptionsItemSelected(final MenuItem item) {
		return super.onOptionsItemSelected(item);
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
    
    /****************************/
    
	//to close this activity
	public void halt()
	{
		mt.terminate();
		this.finish();
	}

	protected void showMyMenu()
    {
    	LayoutInflater layoutInflater 
	     = (LayoutInflater)getBaseContext()
	      .getSystemService(LAYOUT_INFLATER_SERVICE);  
	    LinearLayout popupView =(LinearLayout)layoutInflater.inflate(R.layout.my_opt_menu, null);
	    Button act_btn=(Button)popupView.findViewById(R.id.act_btn);
	    act_btn.setWidth(this.getDisplayWidth()/4);
	    act_btn.setTextColor(Color.DKGRAY);
	    Button sett_btn=(Button)popupView.findViewById(R.id.Sett_btn);
	    sett_btn.setWidth(this.getDisplayWidth()/4);
	    sett_btn.setTextColor(Color.DKGRAY);
	    Button voice_btn=(Button)popupView.findViewById(R.id.voice_btn);
	    voice_btn.setWidth(this.getDisplayWidth()/4);
	    voice_btn.setTextColor(Color.DKGRAY);
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
	    menu.showAtLocation(this.findViewById(R.id.robot_working_global_layout), 0, 0, (main_layout.getMeasuredHeight()-popupView.getHeight()));
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

			}
		});
	    act_btn.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				
			}
		});
	    voice_btn.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {

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
	
	public void changeAction()
	{
		runningAction.setText(myApp.getCurrentTask());
	}
	
	protected final int SETTINGS_CODE=10;
	
	//Method to manage the callback from a start activity for results (i.e. it runs when you change the settings)
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
    	try{
    		super.onActivityResult(requestCode, resultCode, data);
    	}catch(Exception e){
    		Log.i(TAG,"Exception on Activity resut");
    		myApp.unsetSettings();
    	}
    	switch(requestCode){
        	case(SETTINGS_CODE):{
        		if(resultCode==Activity.RESULT_OK)
        		{
        			myApp.unsetSettings();
        			myPreferences.loadPreferences();
        			myApp.updatePreferences();
        			//myApp.restartServices(myPreferences);
        			Log.i("AccompanyGUI","onActivityResult");
        		}
        	} break;
    	}
    }
	
}

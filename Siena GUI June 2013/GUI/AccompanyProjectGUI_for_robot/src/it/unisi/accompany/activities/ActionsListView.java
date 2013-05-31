package it.unisi.accompany.activities;

import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import it.unisi.accompany.AccompanyGUIApp;
import it.unisi.accompany.AccompanyPreferences;
import it.unisi.accompany.R;
import it.unisi.accompany.widget.actionliststuffs.Action;
import it.unisi.accompany.widget.actionliststuffs.ActionButton;
import it.unisi.accompany.widget.actionliststuffs.ActionsEnvironmentButton;
import it.unisi.accompany.widget.actionliststuffs.HorizontalEnvLine;
import android.app.Activity;
import android.content.Intent;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.os.Bundle;
import android.os.StrictMode;
import android.util.Log;
import android.view.Display;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.LinearLayout;
import android.widget.PopupWindow;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.FrameLayout.LayoutParams;

public class ActionsListView extends Activity{
	
	protected final String TAG = "AccompanyGUI - ActionsListView";
	
	protected AccompanyGUIApp myApp;
	protected ActionsListView me;
	
	protected final int SETTINGS_CODE=10;
	
	protected AccompanyPreferences myPreferences;
	
	//layouts and menus
	protected PopupWindow popupWindow;
	protected ImageButton opt_menu;
	
	protected PopupWindow menu;
	protected RelativeLayout main_layout;
	
	//layout stuffs
	protected Button switch_to_user_button;
	protected Button switch_to_robot_button;
	protected LinearLayout environmentsList;
	protected LinearLayout actionsList;
	protected TextView title_tv;
	
	protected String old_environment = null;
	protected ArrayList<String> environments;
	protected ArrayList<ActionsEnvironmentButton> environment_buttons;
	protected ArrayList<Action> actions;
	
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		//standrd things
		super.onCreate(savedInstanceState);
		this.setContentView(R.layout.actions_list);
		
		Log.i(TAG,"on create ActionList");
		
		//Setting up the policies for the use of threads
        if (android.os.Build.VERSION.SDK_INT > 9) {
			StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder()
					.permitAll().build();
			StrictMode.setThreadPolicy(policy);
		}
        //recovering the application that owns this activity
        //and setting the activity variable
        myApp=(AccompanyGUIApp)this.getApplication();
        myApp.setActionsView(this);
        me=this;
        
        //reading the preferences 
        myPreferences=new AccompanyPreferences(this);
        myPreferences.loadPreferences();
		
		switch_to_user_button=(Button)this.findViewById(R.id.switchtouser);
		switch_to_robot_button=(Button)this.findViewById(R.id.switchtorobot);
		environmentsList=(LinearLayout)this.findViewById(R.id.environments_list_view);
		actionsList=(LinearLayout)this.findViewById(R.id.actions_list_view);
		title_tv=(TextView)this.findViewById(R.id.actions_list_title_tv);
		
		switch_to_user_button.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				switchToUserView();
			}});
		switch_to_robot_button.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				switchToRobotView();
			}});
		
		opt_menu=(ImageButton)this.findViewById(R.id.optmenu);
	    opt_menu.setOnClickListener(new View.OnClickListener() {
			
			@Override
			public void onClick(View arg0) {
				showMyMenu();
			}
		});
		
	    
	    main_layout=(RelativeLayout)this.findViewById(R.id.global_actions_list_layout);
		myApp.setRunningActivity(myApp.ACTIONS_VIEW);
		
		//showActions();
		sendRequest();
	}
	
	public void onRestart()
	{
		Log.i("AccompanyGUI","on restart ActionsView ");
		myPreferences.loadPreferences();
		super.onRestart();

		if (old_environment==null)
		{
			//coming here normally: reload the actions, reset the layout
			title_tv.setText(getResources().getString(R.string.actions_list_title));
			actionsList.removeAllViews();
			environmentsList.removeAllViews();
			//showActions();
			sendRequest();
		}
		else
		{
			actionsList.removeAllViews();
			environmentsList.removeAllViews();
			//showActions();
			sendRequest();
			//provo a fare lo show dell'oldenviroment
			if (environments.contains(old_environment))
				this.showActionsFor(old_environment);
		}
		
		//SETTING THIS AS THE ACTIVITY FOCUSED
		myApp.setRunningActivity(myApp.ACTIONS_VIEW);
	}
	
	public void sendRequest()
	{
		//myApp.RequestToDB(myApp.ALL_ACTIONS_REQUEST_CODE,-1);
		myApp.db_client.getFullActionList();
	}
	
	public void sendActionListActionRequest(int id)
	{
		myApp.sendActionRequest(id);
	}
	
	public void handleResponse(String res)
	{
    	if (actions==null) actions = new ArrayList<Action>();
    	else actions.clear();
    	
        //parse json data
        try{
            JSONArray jArray = new JSONArray(res);
            for(int i=0;i<jArray.length();i++){
                JSONObject json_data = jArray.getJSONObject(i);
                Action action= new Action(json_data.getInt("apId"),
                    	json_data.getString("ap_label"),json_data.getDouble("likelihood"),
                    	json_data.getString("phraseal_feedback"),
                    	json_data.getInt("precond_id"),json_data.getString("location_name"));
                actions.add(action);
            }
        }catch(JSONException e){
            Log.e("log_tag", "* Error parsing data "+e.toString());
        }
		
		showActions();
	}
	
	protected void showActions()
	{
		//actions=getAllActionsFromDB();
		
		if (environments!=null) environments.clear();
		else environments= new ArrayList<String>();
		
		if (environment_buttons!=null) environment_buttons.clear();
		else environment_buttons= new ArrayList<ActionsEnvironmentButton>();
		
		for(int i=0; i<actions.size();i++)
    	{
    		if (!environments.contains(actions.get(i).environment))
    			environments.add(actions.get(i).environment);
    	}
		
		for (int i=0;i<environments.size();i++) 
		{
			ActionsEnvironmentButton aeb= new ActionsEnvironmentButton(getApplicationContext(),
					environments.get(i),this);
			LinearLayout.LayoutParams p= new LinearLayout.LayoutParams(
					LinearLayout.LayoutParams.MATCH_PARENT,LinearLayout.LayoutParams.WRAP_CONTENT);
			aeb.setLayoutParams(p);
			environmentsList.addView(aeb);
			environmentsList.measure(getDisplayWidth(), getDisplayHeight());
			Log.e("measure",""+environmentsList.getMeasuredWidth());
			/*environmentsList.addView(new HorizontalEnvLine(getApplicationContext(),
					Color.WHITE,((int)(environmentsList.getMeasuredWidth()*2.5))));*/
			environmentsList.addView(new HorizontalEnvLine(getApplicationContext(),
					Color.WHITE,((int)(environmentsList.getMeasuredWidth()*2.5))));
			environment_buttons.add(aeb);
		}
	}
	
	public void showActionsFor(String e)
	{
		//removig all acurrently shown actions
		actionsList.removeAllViews();
		//resetting the backgrounds of the non-selected buttons
		for (int i=0;i<environment_buttons.size();i++)
			if (!environment_buttons.get(i).getEnvironment().equals(e)) 
					environment_buttons.get(i).setBackgroundColor(Color.TRANSPARENT);
		title_tv.setText(getResources().getString(R.string.actions_list_title)+" for "+ e);
		for (int i=0;i<actions.size();i++)
        {
    		if (actions.get(i).environment.equals(e))
    		{
    			Action a=actions.get(i);
    			//show the button for the Action
    			ActionButton ab= new ActionButton(getApplicationContext()
    						,a.name,a.phrase,this,a.ap_id,a.act_precondition_id);
    				LinearLayout.LayoutParams p= new LinearLayout.LayoutParams(
    						LinearLayout.LayoutParams.WRAP_CONTENT,LinearLayout.LayoutParams.WRAP_CONTENT);
    				ab.setLayoutParams(p);
    				actionsList.addView(ab);		
    		}
        }
		old_environment=e;
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
	    act_btn.setClickable(false);
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
	    menu.showAtLocation(this.findViewById(R.id.global_actions_list_layout), 0, 0, (main_layout.getMeasuredHeight()-popupView.getHeight()));
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
				myApp.setSettings();
				Intent settingsIntent = new Intent(ActionsListView.this,
	    				Settings.class);
	    		ActionsListView.this.startActivityForResult(settingsIntent,SETTINGS_CODE);
	    		menu.dismiss();
			}
		});
	    act_btn.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				//do nothing I're already here!
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
	    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
	    	super.onActivityResult(requestCode, resultCode, data);
	    	switch(requestCode){
	        	case(SETTINGS_CODE):{
	        		if(resultCode==Activity.RESULT_OK)
	        		{
	        			myApp.unsetSettings();
	        			myPreferences.loadPreferences();
	        			myApp.updatePreferences();
	        			//myApp.setImagesRate(myPreferences.getImagesRate());
	        			//Log.i("AccompanyGUI","onActivityResult");
	        		}
	        	} break;
	    	}
	    }
	
	private void setPreferences(boolean speech_mode) {
	    SharedPreferences preferences = getSharedPreferences("accompany_gui_ros",
	        MODE_PRIVATE);
	    SharedPreferences.Editor editor = preferences.edit();

	    editor.putString("ros_master_ip",myPreferences.getRosMasterIP());
	    //Log.e("ACC","imr "+myPreferences.getImagesRate());
	    editor.putInt("images_rate", myPreferences.getImagesRate());
	    editor.putBoolean("speech_mode", speech_mode);
	   
	    editor.commit();
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
	 
	 public void halt()
	 {
		 this.finish();
	 }
	 
	 //SWITCHING VIEWS
	 
	//Switching to robot view (on right banner clicks), the robot view activity is launched:
		public void switchToRobotView()
		{
			
			myApp.StartSubscribing();
			old_environment=null;
			final Intent intent = new Intent().setClass(ActionsListView.this.me, RobotView.class);
			ActionsListView.this.startActivity(intent);
			finish();
		}
		
		//to switch to the user view we need to launch the correct activity
		protected void switchToUserView()
		{
			old_environment=null;
			final Intent intent = new Intent().setClass(ActionsListView.this.me, UserView.class);
			ActionsListView.this.startActivity(intent);
		}
		
		//Switching to the "Robot executing command" view, (called when a Button,i.e. command, is pressed)
		public void showRobotExecutingCommandView()
		{
			myApp.st.setMode(false);
			myApp.StartSubscribing();
			myApp.robotBusy();
			final Intent intent = new Intent().setClass(ActionsListView.this.me, RobotWorkingView.class);
			ActionsListView.this.startActivity(intent);
		}
		
}

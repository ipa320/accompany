package com.questit.accompany2.activities;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
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

import com.questit.accompany2.AccompanyGUI_app2;
import com.questit.accompany2.AccompanyPreferences;
import com.questit.accompany2.R;
import com.questit.accompany2.data.Action;
import com.questit.accompany2.widget.ActionFatherButton;
import com.questit.accompany2.widget.ActionSimpleButton;
import com.questit.accompany2.widget.ActionsEnvironmentButton;
import com.questit.accompany2.widget.HorizontalEnvLine;
import com.questit.accompany2.widget.RobotFatherLabel;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.graphics.Color;
import android.os.Bundle;
import android.os.StrictMode;
import android.util.Log;
import android.view.Display;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.ListView;
import android.widget.PopupWindow;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.FrameLayout.LayoutParams;

public class ActionsListView extends Activity {
	
	protected AccompanyGUI_app2 myApp;
	protected ActionsListView me;
	
	protected final int SETTINGS_CODE=10;
	
	protected ImageButton left_button;
	protected ImageButton right_button;
	protected LinearLayout environmentsList;
	protected LinearLayout actionsList;
	protected TextView title_tv;
	
	protected String old_environment = null;
	
	protected AccompanyPreferences myPreferences;
	
	protected ArrayList<Action> actions;
	protected ArrayList<String> environments;

	protected ArrayList<ActionsEnvironmentButton> environment_buttons;
	
	protected PopupWindow popupWindow;
	
	//on create
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		//standrd things
		super.onCreate(savedInstanceState);
		this.setContentView(R.layout.actions_list);
		
		Log.i("AccompanyGUI","on create ActionList");
		
		//Setting up the policies for the use of threads
        if (android.os.Build.VERSION.SDK_INT > 9) {
			StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder()
					.permitAll().build();
			StrictMode.setThreadPolicy(policy);
		}
        
      //recovering the application that owns this activity
      //and setting the activity variable
      myApp=(AccompanyGUI_app2)this.getApplication();
      myApp.setActionsView(this);
      me=this;
        
		//reading the preferences 
		myPreferences=new AccompanyPreferences(this);
		myPreferences.loadPreferences();
		
		left_button=(ImageButton)this.findViewById(R.id.left_actions_button);
		right_button=(ImageButton)this.findViewById(R.id.right_actions_button);
		environmentsList=(LinearLayout)this.findViewById(R.id.environments_list_view);
		actionsList=(LinearLayout)this.findViewById(R.id.actions_list_view);
		title_tv=(TextView)this.findViewById(R.id.actions_list_title_tv);
		
		left_button.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				switchToUserView();
			}});
		right_button.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				switchToRobotView();
			}});
		
		myApp.setRunningActivity(myApp.ACTIONS_VIEW);
		
		//showActions();
		sendRequest();
	}
	
	public void onRestart()
	{
		Log.i("AccompanyGUI","on restart ActionsView");
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
		myApp.RequestToDB(myApp.ALL_ACTIONS_REQUEST_CODE,-1);
	}
	
	public void sendActionListActionRequest(String command, String phrase)
	{
		myApp.sendActionRequest(command, phrase);
	}
	
	protected int last_father_ap_id=-1;
	ActionFatherButton lastFatherClicked;
	
	public void sendSonRequest(int id,ActionFatherButton rfl)
	{
		last_father_ap_id=id;
		lastFatherClicked=rfl;
		myApp.RequestToDB(myApp.SONS_REQUEST_CODE, id);
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
                Action action= new Action(json_data.getString("ap_label"),json_data.getString("command")
                		,json_data.getString("type_description"),json_data.getString("location_name"),json_data.getInt(
                				"apId"),json_data.getString("phraseal_feedback"));
                actions.add(action);
            }
        }catch(JSONException e){
            Log.e("log_tag", "Error parsing data "+e.toString());
        }
		
		showActions();
	}
	
	public void handleSonResponse(String res,int son)
	{
		if (last_father_ap_id==son)
			lastFatherClicked.setSons(res);
		lastFatherClicked=null;
		last_father_ap_id=-1;
			
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
        		//secondo il tipo visualizza bottone, seekbar o spinner
    			if (a.type!=a.FATHER)
    			{
        			// per ora solo father o simple
    				ActionSimpleButton asb= new ActionSimpleButton(getApplicationContext()
    						,a.label,a.command,a.phrase,this);
    				LinearLayout.LayoutParams p= new LinearLayout.LayoutParams(
    						LinearLayout.LayoutParams.WRAP_CONTENT,LinearLayout.LayoutParams.WRAP_CONTENT);
    				asb.setLayoutParams(p);
    				actionsList.addView(asb);		
    			}
        		else
        		{
        			ActionFatherButton afb= new ActionFatherButton(getApplicationContext()
        					,a.label,this,a.id);
        			LinearLayout.LayoutParams p= new LinearLayout.LayoutParams(
        					LinearLayout.LayoutParams.WRAP_CONTENT,LinearLayout.LayoutParams.WRAP_CONTENT);
        			afb.setLayoutParams(p);
        			actionsList.addView(afb);
        		}
    		}
        }
		old_environment=e;
	}
	
	 /*protected ArrayList<Action> getAllActionsFromDB()
	    {
	    	ArrayList<Action> query_result=new ArrayList<Action>();
	    	
	    	InputStream is=null;
	        String result="";
	        //data to send (in that case the condition is needed only to have the correct ordering by 
	        // likelihood of action possibilities)
	        //ArrayList<NameValuePair> nameValuePairs = new ArrayList<NameValuePair>();
	       // nameValuePairs.add(new BasicNameValuePair("condition","1"));
	        
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
	        	HttpPost hp= new HttpPost("http://"+myPreferences.getDataBaseIP()+"/full_actions_list.php");
	            //HttpPost hp= new HttpPost("http://192.168.1.104/php_labels.php");
	            //hp.setEntity(new UrlEncodedFormEntity(nameValuePairs));
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
	                Action action= new Action(json_data.getString("ap_label"),json_data.getString("command")
	                		,json_data.getString("type_description"),json_data.getString("location_name"),json_data.getInt(
	                				"apId"),json_data.getString("phraseal_feedback"));
	                query_result.add(action);
	            }
	        }catch(JSONException e){
	            Log.e("log_tag", "Error parsing data "+e.toString());
	        }
	        
	    	return query_result;
	    }
*/
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
	public void showRobotExecutingCommandView(String phrase)
	{
		myApp.StartSubscribing();
		myApp.robotBusy();
		final Intent intent = new Intent().setClass(ActionsListView.this.me, RobotWorkingView.class);
		ActionsListView.this.startActivity(intent);
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
			//do nothing I're already here!
		}
		return true;
		case R.id.settings: {
			Intent settingsIntent = new Intent(ActionsListView.this,
    				Settings.class);
    		ActionsListView.this.startActivityForResult(settingsIntent,SETTINGS_CODE);
		}
		return true;
		default: return super.onOptionsItemSelected(item);
		}
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
		//this.setContentView(null);
		this.finish();
	}
	

}

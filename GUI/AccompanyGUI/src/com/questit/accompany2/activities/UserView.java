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
import com.questit.accompany2.widget.CloudFatherButton;
import com.questit.accompany2.widget.CloudSimpleButton;
import com.questit.accompany2.widget.CloudSonButton;
import com.questit.accompany2.widget.RobotFatherLabel;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Color;
import android.graphics.Rect;
import android.os.Bundle;
import android.os.StrictMode;
import android.util.Log;
import android.view.Display;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.AbsoluteLayout;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.LinearLayout;
import android.widget.PopupWindow;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.FrameLayout.LayoutParams;

public class UserView extends Activity{

	protected AccompanyGUI_app2 myApp;
	protected UserView me;
	
	protected final int SETTINGS_CODE=10;	
	
	protected final int MAXUSERACTIONS=5;
	
	protected ImageButton left_button;
	protected ImageButton right_button;
	@SuppressWarnings("deprecation")
	protected AbsoluteLayout my_layout;
	
	protected AccompanyPreferences myPreferences;
	
	//My Buttons (Possible commands!!)
	protected ArrayList<CloudFatherButton> ButtsParent = null;
	protected ArrayList<CloudSimpleButton> ButtsSimple = null;
	protected ArrayList<CloudSonButton>    ButtsSons   = null;
	
	protected PopupWindow popupWindow;
	
	//on create
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		//standrd things
		super.onCreate(savedInstanceState);
		this.setContentView(R.layout.user_view);
		
		Log.i("AccompanyGUI","on create userView");
		
		//Setting up the policies for the use of threads
        if (android.os.Build.VERSION.SDK_INT > 9) {
			StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder()
					.permitAll().build();
			StrictMode.setThreadPolicy(policy);
		}
        
		//reading the preferences 
		myPreferences=new AccompanyPreferences(this);
		myPreferences.loadPreferences();
		
		//recovering the application that owns this activity
		//and setting the activity variable
		myApp=(AccompanyGUI_app2)this.getApplication();
		myApp.setUserView(this);
		me=this;
		
		Log.e("cc","user view!");
		
		//recovering all the layout stuffs
		my_layout=(AbsoluteLayout)this.findViewById(R.id.user_layout);
		left_button=(ImageButton)this.findViewById(R.id.left_user_button);
		right_button=(ImageButton)this.findViewById(R.id.right_user_button);
		
		//Setting the listners for the banners:
		left_button.setOnClickListener(new View.OnClickListener() {			
			@Override
			public void onClick(View v) {
				switchToRobotView();
			}		});	
		right_button.setOnClickListener(new View.OnClickListener() {			
			@Override
			public void onClick(View v) {
				switchToRobotView();
			}		});	
		
		//setting the background of the layout clickable (to reset the parent buttons)
		//i.e. if you click on background and there are son labels shown they will be removed
		my_layout.setOnClickListener(new View.OnClickListener() {			
			@Override
			public void onClick(View v) {
				resetAllButtsFather();
			}		});	
		
		//getting the User action possibilities from the DB:
		//getUserActions();
		//showing them in the layout
		//showLabels();
		
		//Thread that waits the db client to be active and send the request for the labels.
		//after a certain amount of time it exits and gives an error message
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
					else	if (myApp.db_client.isStarted()) sendRequest();
							else  toastMessage("Error: connection with mnaster db service is down!");
				}
			}
		};
		labelsLoader.start();
		
		myApp.setRunningActivity(myApp.USER_VIEW);
	}
	
	protected boolean isClientDbWorking()
	{
		if (myApp.db_client==null) return false;
		else if (myApp.db_client.isStarted()) return true;
			else return false;
	}
	
	@Override
	public void onRestart()
	{
		Log.i("AccompanyGUI","on restart userView");
		myPreferences.loadPreferences();
		super.onRestart();
		//removing old labels
		removeAllLabels();
		//getting the User action possibilities from the DB:
		//getUserActions();
		//showing them in the layout
		//showLabels();
		if (myApp.db_client==null) toastMessage("Error: cannot connect to the Database service!");
		else	if (myApp.db_client.isStarted()) sendRequest();
				else  toastMessage("Error: connection with mnaster db service is down!");
		//SETTING THIS AS THE ACTIVITY FOCUSED
		myApp.setRunningActivity(myApp.USER_VIEW);
	}
	
	public void sendRequest()
	{
		myApp.RequestToDB(myApp.USER_ACTIONS_REQUEST_CODE,-1);
	}
	
	protected int last_father_ap_id=-1;
	CloudFatherButton lastFatherClicked;
	
	public void sendSonRequest(int id,CloudFatherButton rfl)
	{
		last_father_ap_id=id;
		lastFatherClicked=rfl;
		myApp.RequestToDB(myApp.SONS_REQUEST_CODE, id);
	}
	
	public void handleSonResponse(String res, int son)
	{
		if (last_father_ap_id==son)
			lastFatherClicked.setSons(res);
		lastFatherClicked=null;
		last_father_ap_id=-1;
	}
	
	public void handleResponse(String res)
	{
		if (ButtsParent==null) ButtsParent= new ArrayList<CloudFatherButton>();
    	else ButtsParent.clear();
    	if (ButtsSimple==null) ButtsSimple= new ArrayList<CloudSimpleButton>();
    	else ButtsSimple.clear();
    	if (ButtsSons==null) ButtsSons= new ArrayList<CloudSonButton>();
    	else ButtsSons.clear();
    	
    	Log.e("user response",res);
    	
    	//parse json data
        try{
            JSONArray jArray = new JSONArray(res);
            for(int i=0;i<jArray.length();i++){
                JSONObject json_data = jArray.getJSONObject(i);
                if ((json_data.getString("type_description").contains("Simple"))||(json_data.getString("type_description").contains("simple")))
                ButtsSimple.add(new CloudSimpleButton(getApplicationContext(),
                		json_data.getString("ap_label"),
                		json_data.getString("command"),json_data.getDouble("likelihood"),
                		json_data.getString("phraseal_feedback"),
                		this));
                else
                	 ButtsParent.add(new CloudFatherButton(getApplicationContext(),
                			 json_data.getInt("apId"),
                     		json_data.getString("ap_label"),
                     		json_data.getDouble("likelihood"),my_layout,
                     		 this));
            }
        }catch(JSONException e){
            Log.e("log_tag", "Error parsing data "+e.toString());
        }
        
        filterUserActions();
        
        showLabels();
	}
	
	public void sendUserActionRequest(String command, String phrase)
	{
		myApp.sendActionRequest(command, phrase);
	}
	
	@Override
	public void onStart()
	{
		Log.i("AccompanyGUI","on start userView");
		super.onStart();
		//removing old labels
		/*removeAllLabels();
		//getting the User action possibilities from the DB:
		getUserActions();
		//showing them in the layout
		showLabels();
		//SETTING THIS AS THE ACTIVITY FOCUSED
		//myApp.setRunningActivity(myApp.USER_VIEW);
		myApp.setUserView(this);*/
	}
	
	@Override
	public void onResume()
	{
		Log.i("AccompanyGUI","on resume userView");
		super.onResume();
	/*	//removing old labels
		removeAllLabels();
		//getting the User action possibilities from the DB:
		getUserActions();
		//showing them in the layout
		showLabels();
		//SETTING THIS AS THE ACTIVITY FOCUSED
		myApp.setRunningActivity(myApp.USER_VIEW);
		//myApp.setUserView(this);*/
	}
	
	//The method to show the labels on my layout:
	protected void showLabels()
	{
		//this method avoid the overlay!! (Except for the sons)
		boolean done=false;
    	int attemps=0;
    	
    	while (!done)
    		if ((!ButtsSimple.isEmpty())||(!ButtsParent.isEmpty()))
    		{
    			done=true;
    			Display d=getWindowManager().getDefaultDisplay();
    			//int x=(int)((d.getWidth()/2)-ButtsSimple.get(0).getMyWidth()/2);
    			//int y=(int)((d.getHeight()/2)-ButtsSimple.get(0).getMyHeight()/2);
    			int x;
    			int y;
    			AbsoluteLayout.LayoutParams par;//= new AbsoluteLayout.LayoutParams(AbsoluteLayout.LayoutParams.WRAP_CONTENT, AbsoluteLayout.LayoutParams.WRAP_CONTENT, x, y);
    			//ButtsSimple.get(0).setLayoutParams(par);
    			//bigL.addView(ButtsSimple.get(0));    		
    			for (int i=0;i<ButtsSimple.size();i++)
    			{
    				boolean flag=true;
    				while(flag==true)
    				{
    					attemps++;
    					flag=false;
    					x = getBannerWidth()+ (int)(Math.random()*((d.getWidth()-2*getBannerWidth())-ButtsSimple.get(i).getMyWidth()));  //xhdpi
    					y = (int)(Math.random()*((d.getHeight())-ButtsSimple.get(i).getMyHeight()));
    					//toastMessage(x + "    " +y);
    					//AbsoluteLayout.LayoutParams par= new AbsoluteLayout.LayoutParams(Butt.get(i).getWidth(), Butt.get(i).getHeight(), x, y);
    					par= new AbsoluteLayout.LayoutParams(AbsoluteLayout.LayoutParams.WRAP_CONTENT, AbsoluteLayout.LayoutParams.WRAP_CONTENT, x, y);
    					ButtsSimple.get(i).setLayoutParams(par);
    					my_layout.addView(ButtsSimple.get(i));
    					Rect r1;
    					int x1=((AbsoluteLayout.LayoutParams)ButtsSimple.get(i).getLayoutParams()).x;
    					int y1=((AbsoluteLayout.LayoutParams)ButtsSimple.get(i).getLayoutParams()).y;
    					r1= new Rect(x1,y1,(x1+ButtsSimple.get(i).getMyWidth()),(y1+ButtsSimple.get(i).getMyHeight()));
    					//r1= new Rect(x1,y1,(x1+Butt.get(i).getLayoutParams().width),(y1+Butt.get(i).getMyHeight()));
    					Log.i("Info","i:"+i +" rect 1:"+r1.left+", "+r1.top+", "+r1.right+", "+r1.bottom);
    					for (int h=0;h<i;h++)
    					{
    						
    						Rect r2;
    						int x2=((AbsoluteLayout.LayoutParams)ButtsSimple.get(h).getLayoutParams()).x;
    						int y2=((AbsoluteLayout.LayoutParams)ButtsSimple.get(h).getLayoutParams()).y;
    						r2= new Rect(x2,y2,(x2+ButtsSimple.get(h).getMyWidth()),(y2+ButtsSimple.get(h).getMyHeight()));
    						//r2= new Rect(x2,y2,(x2+Butt.get(h).getLayoutParams().getMyWidth()),(y2+Butt.get(h).getMyHeight()));
    						//act.toastMessage(""+Butt.get(h).getLayoutParams().width);
    						Log.i("Info","h:"+h +" rect 2:"+r2.left+", "+r2.top+", "+r2.right+", "+r2.bottom);
    						if (Rect.intersects(r1, r2)) 
    						{
    							flag= true;
    							Log.i("Info","rectangles "+i+" and "+h+" intersecate!");
    						}	
    					}
    			
    					if (flag) 
    					{
    						my_layout.removeView(ButtsSimple.get(i));
    						//act.toastMessage("evviva");
    						if (attemps>15)
        					{
        						done=false;
        						flag=false;
        						for (int k=0;k<i;k++)
        							my_layout.removeView(ButtsSimple.get(k));
        						i=ButtsSimple.size();
        						attemps=0;
        					}
    					}	
    				}
    			}   
    		
    			if (done) 
    				for (int i=0;i<ButtsParent.size();i++)
    				{
    					boolean flag=true;
    					while(flag==true)
    					{
    						attemps++;
    						flag=false;
    						x = getBannerWidth()+ (int)(Math.random()*((d.getWidth()-2*getBannerWidth())-ButtsParent.get(i).getMyWidth()));
    						y = (int)(Math.random()*(d.getHeight()-ButtsParent.get(i).getMyHeight()));
    						ButtsParent.get(i).setPosition(x, y);
    						//toastMessage(x + "    " +y);
    						//AbsoluteLayout.LayoutParams par= new AbsoluteLayout.LayoutParams(Butt.get(i).getWidth(), Butt.get(i).getHeight(), x, y);
    						par= new AbsoluteLayout.LayoutParams(AbsoluteLayout.LayoutParams.WRAP_CONTENT, AbsoluteLayout.LayoutParams.WRAP_CONTENT, x, y);
    						ButtsParent.get(i).setLayoutParams(par);
    						my_layout.addView(ButtsParent.get(i));
    						Rect r1;
    						int x1=((AbsoluteLayout.LayoutParams)ButtsParent.get(i).getLayoutParams()).x;
    						int y1=((AbsoluteLayout.LayoutParams)ButtsParent.get(i).getLayoutParams()).y;
    						r1= new Rect(x1,y1,(x1+ButtsParent.get(i).getMyWidth()),(y1+ButtsParent.get(i).getMyHeight()));
    						//r1= new Rect(x1,y1,(x1+Butt.get(i).getLayoutParams().width),(y1+Butt.get(i).getMyHeight()));
    						Log.i("Info","i:"+i +" rect 1:"+r1.left+", "+r1.top+", "+r1.right+", "+r1.bottom);
    						for (int h=0;h<i;h++)
    						{
    							Rect r2;
    							int x2=((AbsoluteLayout.LayoutParams)ButtsParent.get(h).getLayoutParams()).x;
    							int y2=((AbsoluteLayout.LayoutParams)ButtsParent.get(h).getLayoutParams()).y;
    							r2= new Rect(x2,y2,(x2+ButtsParent.get(h).getMyWidth()),(y2+ButtsParent.get(h).getMyHeight()));
    							//r2= new Rect(x2,y2,(x2+Butt.get(h).getLayoutParams().getMyWidth()),(y2+Butt.get(h).getMyHeight()));
    							//act.toastMessage(""+Butt.get(h).getLayoutParams().width);
    							Log.i("Info","h:"+h +" rect 2:"+r2.left+", "+r2.top+", "+r2.right+", "+r2.bottom);
    							if (Rect.intersects(r1, r2)) 
    							{
    								flag= true;
    								Log.i("Info","rectangles "+i+" and "+h+" intersecate!");
    							}
    						}	
    						for (int h=0;h<ButtsSimple.size();h++)
    						{
    							Rect r2;
    							int x2=((AbsoluteLayout.LayoutParams)ButtsSimple.get(h).getLayoutParams()).x;
    							int y2=((AbsoluteLayout.LayoutParams)ButtsSimple.get(h).getLayoutParams()).y;
    							r2= new Rect(x2,y2,(x2+ButtsSimple.get(h).getMyWidth()),(y2+ButtsSimple.get(h).getMyHeight()));
    							//r2= new Rect(x2,y2,(x2+Butt.get(h).getLayoutParams().getMyWidth()),(y2+Butt.get(h).getMyHeight()));
    							//act.toastMessage(""+Butt.get(h).getLayoutParams().width);
    							Log.i("Info","h:"+h +" rect 2:"+r2.left+", "+r2.top+", "+r2.right+", "+r2.bottom);
    							if (Rect.intersects(r1, r2)) 
    							{
    								flag= true;
    								Log.i("Info","rectangles "+i+" and "+h+" intersecate!");
    							}
    						}
    						if (flag) 
    						{
    							my_layout.removeView(ButtsParent.get(i));
    							//act.toastMessage("evviva");
    							if (attemps>15)
    							{
        							done=false;
        							flag=false;
        							for (int k=0;k<ButtsSimple.size();k++)
        								my_layout.removeView(ButtsSimple.get(k));
        							for (int k=0;k<i;k++)
        								my_layout.removeView(ButtsParent.get(k));
        							i=ButtsParent.size();
        							attemps=0;
    							}
    						}
    					}
    				} 
    		}
    		else
    		{
    			done=true;
    		}
	}
	
	//The method to remove the labels (will be usefull only in case of refresh or in case of 
	//restart of the activity) (I don't think it will be used actually)
	protected void removeAllLabels()
	{
		if (ButtsSons!=null)
		{
			for (int i=0;i<ButtsSons.size();i++)
				my_layout.removeView(ButtsSons.get(i));
			ButtsSons.clear();
		}
		if (ButtsParent!=null)
		{
			for (int i=0;i<ButtsParent.size();i++)
				my_layout.removeView(ButtsParent.get(i));
			ButtsParent.clear();
		}
		if (ButtsSimple!=null)
		{
			for (int i=0;i<ButtsSimple.size();i++)
				my_layout.removeView(ButtsSimple.get(i));
			ButtsSimple.clear();
		}
		
	}
	
	//The method to get the action possibilities from the robot
	/*protected void getUserActions()
	{
		if (ButtsParent==null) ButtsParent= new ArrayList<CloudFatherButton>();
    	else ButtsParent.clear();
    	if (ButtsSimple==null) ButtsSimple= new ArrayList<CloudSimpleButton>();
    	else ButtsSimple.clear();
    	if (ButtsSons==null) ButtsSons= new ArrayList<CloudSonButton>();
    	else ButtsSons.clear();
    	
    	InputStream is=null;
        String result="";
        //data to send
        ArrayList<NameValuePair> nameValuePairs = new ArrayList<NameValuePair>();  
        nameValuePairs.add(new BasicNameValuePair("location",this.getLocation()));
        
        
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
        	HttpPost hp= new HttpPost("http://"+myPreferences.getDataBaseIP()+"/user_action_possibilities.php");
        	Log.i("AccompanyGUI","connecting to: http://"+myPreferences.getDataBaseIP()+"/user_action_possibilities.php");
            //HttpPost hp= new HttpPost("http://"+CareOBotIP+"/olds/my_uh_user_APs.php");
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
                if ((json_data.getString("type_description").contains("Simple"))||(json_data.getString("type_description").contains("simple")))
                ButtsSimple.add(new CloudSimpleButton(getApplicationContext(),
                		json_data.getString("ap_label"),
                		json_data.getString("command"),json_data.getDouble("likelihood"),
                		json_data.getString("phraseal_feedback"),
                		this,myPreferences.getRobotIP(),myPreferences.getRobotPort()));
                else
                	 ButtsParent.add(new CloudFatherButton(getApplicationContext(),
                			 json_data.getInt("apId"),
                     		json_data.getString("ap_label"),
                     		json_data.getDouble("likelihood"),my_layout,
                     		myPreferences.getRobotIP(), this,myPreferences.getRobotPort(),
                			 myPreferences.getDataBaseIP()));
            }
        }catch(JSONException e){
            Log.e("log_tag", "Error parsing data "+e.toString());
        }
        
        filterUserActions();
	}*/
	
	 protected void filterUserActions()  //assumes buttons order by Likelihood
	    {
	    	Log.i("AccompanyGUI","Before filtering, Buttons array dimensions are: -parent: "+ ButtsParent.size());
	    	Log.i("AccompanyGUI","                                                .simple: "+ ButtsSimple.size());
	    	while(ButtsParent.size()+ButtsSimple.size()>MAXUSERACTIONS)
	    	{
	    		if (ButtsParent.get(ButtsParent.size()-1).getLikelihood() < 
	    				ButtsSimple.get(ButtsSimple.size()-1).getLikelihood())
	    			ButtsParent.remove(ButtsParent.size()-1);
	    		else
	    			ButtsSimple.remove(ButtsSimple.size()-1);
	    	}
	    }
	
	//The method to recover my location
	//Now it's hard Coded, It will be recover from the db
	public String getLocation()
	{
		return "502";
	}
	//This is needed in order to reset the Father Buttons which are displaying their sons:
	//i.e. to remove sons when the outside is pressed
	public void resetAllButtsFather()
	{
		if (ButtsParent!=null)
			for (int h=0; h<ButtsParent.size();h++)
				ButtsParent.get(h).resetSons();
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
		myApp.StartSubscribing();
		myApp.robotBusy();
		final Intent intent = new Intent().setClass(UserView.this.me, RobotWorkingView.class);
		UserView.this.startActivity(intent);
		finish();
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
    
    /****************************/
    /*  Some utilities methods  */
    /****************************/
    
    public int getBannerWidth()
    {
    	left_button.measure(getWindowManager().getDefaultDisplay().getWidth(), 
    			getWindowManager().getDefaultDisplay().getHeight());
    	//Log.i("AccompanyGUI","banner width: "+left_button.getMeasuredWidth());
    	return left_button.getMeasuredWidth();
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
			final Intent intent = new Intent().setClass(UserView.this.me, ActionsListView.class);
			UserView.this.startActivity(intent);
			finish();
		}
		return true;
		case R.id.settings: {
			Intent settingsIntent = new Intent(UserView.this,
    				Settings.class);
    		UserView.this.startActivityForResult(settingsIntent,SETTINGS_CODE);
		}
		default: return super.onOptionsItemSelected(item);
		}
    }
    
    public AccompanyPreferences getPreferences()
    {
    	return myPreferences;
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
        			Log.i("AccompanyGUI","onActivityResult");
        		}
        	} break;
    	}
    }
    
    /****************************/
    
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
    
	//to close this activity
	public void halt()
	{
		//this.setContentView(null);
		this.finish();
		this.onDestroy();
	}
}

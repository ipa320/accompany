package com.questit.accompany2.activities;

import com.questit.accompany2.AccompanyGUI_app2;
import com.questit.accompany2.AccompanyPreferences;
import com.questit.accompany2.R;

import android.app.Activity;
import android.app.ProgressDialog;
import android.content.Intent;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.Display;
import android.view.Gravity;
import android.view.KeyEvent;
import android.view.LayoutInflater;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.PopupWindow;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.FrameLayout.LayoutParams;

public class LoginPage extends Activity{
	
	LoginPage me;
	protected PopupWindow popupWindow;
	protected AccompanyGUI_app2 myApp;
	
	//protected EditText pwd;
	protected EditText rosIp;
	protected EditText usr;
	protected Button ok;
	
	protected final int SETTINGS_CODE=10;
	
	protected boolean waited_flag,clicked;
	protected String user,password;
	
	protected AccompanyPreferences myPreferences;
	protected Thread labelsLoader;
	protected Handler h;
	protected String Ip;
	
	public ProgressDialog pd;
	
	//aggiungi option per cambiare ip del db!!
	
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		//standrd things
		super.onCreate(savedInstanceState);
		this.setContentView(R.layout.login_page);
		
		h= new Handler();
		
		//reading the preferences 
		myPreferences=new AccompanyPreferences(this);
		myPreferences.loadPreferences();
		
		waited_flag=false;
		clicked=false;
		me=this;
		myApp=(AccompanyGUI_app2)this.getApplication();
	    myApp.setLoginPage(this);
	    
	    ok=(Button)this.findViewById(R.id.ok_btn_login);
	    //pwd=(EditText)this.findViewById(R.id.password_et);
	    rosIp=(EditText)this.findViewById(R.id.ip_et);
	    usr=(EditText)this.findViewById(R.id.user_et);
	    
	    rosIp.setText(myPreferences.getRosMasterIP());
	    
	    rosIp.setOnEditorActionListener(new TextView.OnEditorActionListener() {
			
			@Override
			public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
				if(actionId==EditorInfo.IME_NULL && event.getAction()==KeyEvent.ACTION_DOWN)
				{
					Ip=rosIp.getText().toString();
					user=usr.getText().toString();
					clicked=true;
					/*if (isClientDbWorking())
					{
						Log.e("Accompany-pwd","u: "+user+" p: "+password);
						myApp.db_client.login(user, password);
					}
					else if (waited_flag) toastMessage("Cannot connect with ROS master and DB... please go to settings.");
						 else toastMessage("Loading... please wait.");
						 */
					startLoading();
					if (!Ip.equals(myPreferences.getRosMasterIP()))
						setPreferences();
				}
				return false;
			}
		});
	    usr.setOnEditorActionListener(new TextView.OnEditorActionListener() {
			
			@Override
			public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
				if(actionId==EditorInfo.IME_NULL && event.getAction()==KeyEvent.ACTION_DOWN)
				{
					Ip=rosIp.getText().toString();
					user=usr.getText().toString();
					clicked=true;
					/*if (isClientDbWorking())
					{
						Log.e("Accompany-pwd","u: "+user+" p: "+password);
						myApp.db_client.login(user, password);
					}
					else if (waited_flag) toastMessage("Cannot connect with ROS master and DB... please go to settings.");
						 else toastMessage("Loading... please wait.");
						 */
					startLoading();
					if (!Ip.equals(myPreferences.getRosMasterIP()))
						setPreferences();
				}
				return false;
			}
		});
	    
	    ok.setOnClickListener(new View.OnClickListener() {
			
			@Override
			public void onClick(View v) {
				//password=pwd.getText().toString();
				Ip=rosIp.getText().toString();
				user=usr.getText().toString();
				clicked=true;
				/*if (isClientDbWorking())
				{
					Log.e("Accompany-pwd","u: "+user+" p: "+password);
					myApp.db_client.login(user, password);
				}
				else if (waited_flag) toastMessage("Cannot connect with ROS master and DB... please go to settings.");
					 else toastMessage("Loading... please wait.");
					 */
				startLoading();
				if (!Ip.equals(myPreferences.getRosMasterIP()))
					setPreferences();
			}
		});
	    /*labelsLoader = new Thread() {
			@Override
			public void run()
			{
				final int max_time=5000;  //5 secondi di attesa del servizo
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
					waited_flag=true;
					if (clicked)
						if (isClientDbWorking())
						{
							myApp.db_client.login(user, password);
						}
						else
							h.post(new Runnable(){
								@Override
								public void run() {
									me.toastMessage("Cannot connect with ROS master and DB... please go to settings.");
								}});					
				}
			}
		};
		labelsLoader.start();*/
	}
	
	 private void setPreferences() {
		    SharedPreferences preferences = getSharedPreferences("accompany_gui_ros",
		        MODE_PRIVATE);
		    SharedPreferences.Editor editor = preferences.edit();

		    editor.putString("ros_master_ip",Ip);
		    //Log.e("ACC","imr "+myPreferences.getImagesRate());
		    editor.putInt("images_rate", myPreferences.getImagesRate());
		   
		    editor.commit();
		  }
	
	protected void startLoading()
	{
		myApp.startServices(Ip,myPreferences.getImagesRate());
		pd= ProgressDialog.show(this, "Accompany GUI", "Connecting to ROS Master...");
		
		labelsLoader = new Thread() {
			@Override
			public void run()
			{
				final int max_time=15000;  //5 secondi di attesa del servizo
				int waited=0;
				try{
					while ((waited < max_time)&&(!isClientDbWorking()))
					{
						Thread.sleep(100);
						waited+=100;
					}
				} catch(final Exception e){
					
				}
				finally{  //if service is active send request to db
					waited_flag=true;
					if (clicked)
						if (isClientDbWorking())
						{
							myApp.db_client.login(user, password);
						}
						else
						{
							h.post(new Runnable(){
								@Override
								public void run() {
									me.toastMessage("Cannot connect with ROS master and DB. Please check the ROS Master Ip...");
									closeAppOnError();
								}});
						}
					pd.dismiss();
				}
			}
		};
		labelsLoader.start();
		
	}
	
	protected boolean isClientDbWorking()
	{
		if (myApp.db_client==null) return false;
		else if (myApp.db_client.isStarted()) return true;
			else return false;
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
		inflater.inflate(R.menu.optmenu_login, menu);
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
		case R.id.settings: {
			Intent settingsIntent = new Intent(LoginPage.this,
    				Settings.class);
			//settingsIntent.putExtra("login", true);
    		LoginPage.this.startActivityForResult(settingsIntent,SETTINGS_CODE);
		}return true;
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
    
    public void loginResult(String s)
    {
    	if (s.equals("-1"))
    		toastMessage("Wrong username or password!");
    	else
    	{
    		myApp.setUserId(Integer.parseInt(s));
    		final Intent intent = new Intent().setClass(LoginPage.this.me, UserView.class);
			LoginPage.this.startActivity(intent);
			finish();
    	}
    }
    
    @Override
    public void onDestroy()
    {
    	//this.setContentView(null);
    	super.onDestroy();
    	
    }
    
    //Method to manage the callback from a start activity for results (i.e. it runs when you change the settings)
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
    	super.onActivityResult(requestCode, resultCode, data);
    	switch(requestCode){
        	case(SETTINGS_CODE):{
        		if(resultCode==Activity.RESULT_OK)
        		{
        			/*if (labelsLoader!=null) labelsLoader.interrupt();
        			labelsLoader = new Thread() {
        				@Override
        				public void run()
        				{
        					final int max_time=5000;  //5 secondi di attesa del servizo
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
        						waited_flag=true;
        						if (clicked)
        							if (isClientDbWorking())
        							{
        								myApp.db_client.login(user, password);
        							}
        							else
        								h.post(new Runnable(){
        									@Override
        									public void run() {
        										me.toastMessage("Cannot connect with ROS master and DB... please go to settings.");
        									}});					
        					}
        				}
        			};
        			labelsLoader.start();*/
        			myPreferences.loadPreferences();
        			rosIp.setText(myPreferences.getRosMasterIP());
        			//myApp.restartServices(myPreferences);
        			Log.i("AccompanyGUI","onActivityResult");
        		}
        	} break;
    	}
    }

    public void closeAppOnError()
	{
		final ProgressDialog pd= ProgressDialog.show(this, "Accompany GUI", "Wrong Ros Master Ip! Shutdown...");
		pd.setIcon(android.R.drawable.ic_dialog_alert);
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
					pd.dismiss();
					myApp.closeApp();
				}
			}
		};
		waiter.start();
	}
}

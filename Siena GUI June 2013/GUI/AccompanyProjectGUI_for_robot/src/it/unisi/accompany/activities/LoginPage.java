package it.unisi.accompany.activities;

import it.unisi.accompany.AccompanyGUIApp;
import it.unisi.accompany.AccompanyPreferences;
import it.unisi.accompany.R;

import java.io.IOException;

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
import android.widget.ImageButton;
import android.widget.LinearLayout;
import android.widget.PopupWindow;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.FrameLayout.LayoutParams;

import com.hoho.android.usbserial.driver.UsbSerialProber;


public class LoginPage extends Activity{
	
	LoginPage me;
	protected PopupWindow popupWindow;
	protected AccompanyGUIApp myApp;
	
	//protected EditText pwd;
	protected EditText rosIp;
	protected EditText usr;
	protected Button ok;
	
	protected RelativeLayout mainLayout;
	
	protected final int SETTINGS_CODE=10;
	
	protected boolean waited_flag,clicked;
	protected String user,password;
	
	protected AccompanyPreferences myPreferences;
	protected Thread labelsLoader;
	protected Handler h;
	protected String Ip;
	
	protected ImageButton opt_menu;
	
	public ProgressDialog pd=null;
	int waited=0;
	
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
		myApp=(AccompanyGUIApp)this.getApplication();
	    myApp.setLoginPage(this);
	    
	    mainLayout=(RelativeLayout)this.findViewById(R.id.login_main);
	    ok=(Button)this.findViewById(R.id.ok_btn_login);
	    //pwd=(EditText)this.findViewById(R.id.password_et);
	    rosIp=(EditText)this.findViewById(R.id.ip_et);
	    usr=(EditText)this.findViewById(R.id.user_et);
	    
	    rosIp.setText(myPreferences.getRosMasterIP());
	    opt_menu=(ImageButton)this.findViewById(R.id.optmenu);
	    opt_menu.setOnClickListener(new View.OnClickListener() {
			
			@Override
			public void onClick(View arg0) {
				showMyMenu();
			}
		});
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
					//if (!Ip.equals(myPreferences.getRosMasterIP()))
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
					//if (!Ip.equals(myPreferences.getRosMasterIP()))
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
				//if (!Ip.equals(myPreferences.getRosMasterIP()))
				setPreferences();
			}
		});
	}
	
	@Override
	public void onResume()
	{
		super.onResume();
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
        if (myApp.getRunning()!=-1)
        {
        	switch (myApp.getRunning())
			{
				case 2://myApp.ROBOT_VIEW:
				{
					final Intent intent = new Intent().setClass(LoginPage.this.me, RobotView.class);
					LoginPage.this.startActivity(intent);
					finish();
				} break;
				case 1://myApp.USER_VIEW:
				{
					final Intent intent = new Intent().setClass(LoginPage.this.me, UserView.class);
					LoginPage.this.startActivity(intent);
					finish();
				} break;
				case 3://myApp.ACTIONS_VIEW:
				{
					final Intent intent = new Intent().setClass(LoginPage.this.me, ActionsListView.class);
					LoginPage.this.startActivity(intent);
					finish();
				} break;
				case 4://myApp.EXECUTE_VIEW:
				{
					final Intent intent = new Intent().setClass(LoginPage.this.me, RobotWorkingView.class);
					LoginPage.this.startActivity(intent);
					finish();
				}
			}
        }
	}
	
	 private void setPreferences() {
		    SharedPreferences preferences = getSharedPreferences("accompany_gui_ros",
		        MODE_PRIVATE);
		    SharedPreferences.Editor editor = preferences.edit();

		    editor.putString("ros_master_ip",Ip);
		    //Log.e("ACC","imr "+myPreferences.getImagesRate());
		    //editor.putInt("images_rate", myPreferences.getImagesRate());
		    editor.putBoolean("speech_mode", myPreferences.getSpeechMode());
		    editor.putString("database_ip", myPreferences.getDatabaseIp());
	        editor.putString("database_port", myPreferences.getDatabasePort());
	        
		    editor.commit();
		  }
	
	protected void startLoading()
	{
		//myApp.startServices(Ip,myPreferences.getImagesRate());
		myApp.SetIp(Ip);
		if (pd!=null) pd.dismiss();
		pd= ProgressDialog.show(this, this.getResources().getString(R.string.gui_title), this.getResources().getString(R.string.connecting_master));
		
		labelsLoader = new Thread() {
			@Override
			public void run()
			{
				final int max_time=5000;  //5 secondi di attesa del servizo
				waited=0;
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
							h.post(new Runnable(){
								@Override
								public void run() {
									me.toastMessage(LoginPage.this.getResources().getString(R.string.cannot_connect_master));
								}});
					pd.dismiss();
				}
			}
		};
		labelsLoader.start();
		
	}
	
	protected boolean isClientDbWorking()
	{
		/*if (myApp.db_client==null) return false;
		else if (myApp.db_client.isStarted()) return true;
			else return false;*/
		if (myApp.db_client==null) return false;
		else return true;
	}
	
	 //Toast to send a short message on  the screen
    public Toast toastMessage(String msg)
	{
    	Log.i("AccompanyGUI-LoginPage","toast login" + msg);
		   	CharSequence connessione = msg;
			int duration = Toast.LENGTH_LONG;
			Toast toast = Toast.makeText(this, connessione, duration);
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
       	ptv.setText(this.getResources().getString(R.string.close_act_title));
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
       	ptv2.setText(this.getResources().getString(R.string.are_you_sure_question));
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
       	yes.setText(this.getResources().getString(R.string.yes));
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
       	no.setText(this.getResources().getString(R.string.no));
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
    		toastMessage(this.getResources().getString(R.string.wrong_usr_pwd));
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
        			myApp.unsetSettings();
        			myPreferences.loadPreferences();
        			rosIp.setText(myPreferences.getRosMasterIP());
        			myApp.updatePreferences();
        			//myApp.restartServices(myPreferences);
        			Log.i("AccompanyGUI","onActivityResult");
        		}
        	} break;
    	}
    }

    public void closeAppOnError(String msg)
	{
    	//if (pd!=null) pd.dismiss();
    	//waited=1000000;
		final ProgressDialog pdd= ProgressDialog.show(this, this.getResources().getString(R.string.gui_title), msg);
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
					myApp.closeApp();
				}
			}
		};
		waiter.start();
	}
    
    protected void showMyMenu()
    {
    	LayoutInflater layoutInflater 
	     = (LayoutInflater)getBaseContext()
	      .getSystemService(LAYOUT_INFLATER_SERVICE);  
	    LinearLayout popupView =(LinearLayout)layoutInflater.inflate(R.layout.my_login_opt_menu, null);
	    Button sett_btn=(Button)popupView.findViewById(R.id.Sett_btn);
	    sett_btn.setWidth(this.getDisplayWidth()/2);
	    Button close_btn=(Button)popupView.findViewById(R.id.Close_btn);
	    close_btn.setWidth(this.getDisplayWidth()/2);
	    PopupWindow menu= new PopupWindow(popupView,LayoutParams.MATCH_PARENT,100,true);
	    menu.setContentView(popupView);
		menu.setOutsideTouchable(true);
       	menu.setBackgroundDrawable(getResources().getDrawable(R.drawable.transparent));
	    Log.e("aa",mainLayout.getMeasuredHeight()+ " "+popupView.getHeight());
	    menu.showAtLocation(this.findViewById(R.id.login_main), 0, 0, (mainLayout.getMeasuredHeight()-popupView.getHeight()));
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
	       	ptv.setText(LoginPage.this.getResources().getString(R.string.close_act_title));
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
	       	ptv2.setText(LoginPage.this.getResources().getString(R.string.are_you_sure_question));
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
	       	yes.setText(LoginPage.this.getResources().getString(R.string.yes));
	       	//yes.setBackgroundColor(Color.BLACK);
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
	       	no.setText(LoginPage.this.getResources().getString(R.string.no));
	       	no.setWidth(125);
	       	//yes.setBackgroundColor(Color.BLACK);
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
			}
		});
	    sett_btn.setOnClickListener(new View.OnClickListener() {
			
			@Override
			public void onClick(View v) {
				myApp.setSettings();
				Intent settingsIntent = new Intent(LoginPage.this,
	    				Settings.class);
				//settingsIntent.putExtra("login", true);
	    		LoginPage.this.startActivityForResult(settingsIntent,SETTINGS_CODE);
			}
		});
    }
}
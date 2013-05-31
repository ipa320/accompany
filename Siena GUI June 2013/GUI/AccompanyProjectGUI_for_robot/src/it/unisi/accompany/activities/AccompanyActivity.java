package it.unisi.accompany.activities;

import it.unisi.accompany.AccompanyGUIApp;
import it.unisi.accompany.AccompanyPreferences;
import it.unisi.accompany.clients.DatabaseClient;
import it.unisi.accompany.widget.ActionPossibilityWidget;
import android.app.Activity;
import android.content.Intent;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.os.Bundle;
import android.util.Log;
import android.view.Display;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.MenuInflater;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.PopupWindow;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.FrameLayout.LayoutParams;

public abstract class AccompanyActivity extends Activity{
	
	protected final int SETTINGS_CODE=10;	
	
	protected AccompanyGUIApp myApp;
	
	public AccompanyPreferences myPreferences;
	
	private ActionPossibilityWidget lastApClicked;
	
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
		
		//reading the preferences 
		myPreferences=new AccompanyPreferences(this);
		myPreferences.loadPreferences();
		
		myApp=(AccompanyGUIApp)this.getApplication();
	}
    
	//LOGIC UTILITIES
	
	public abstract void commandRunning(boolean b);
	
	public abstract void removeAllLabels();
	
	public abstract void showLabels();
		
	public abstract void filterUserActions();
	
    public abstract void resetAllActionsPossibilities();
    
    public abstract void resetAllActionsPossibilities(ActionPossibilityWidget apw);
    
    //DB AND APP UTILITIES
    
    public AccompanyGUIApp getMyApp()
    {
    	return myApp;
    }
    
    public DatabaseClient getDbClient()
    {
    	return myApp.db_client;
    }
    
    public void setClicked(ActionPossibilityWidget a)
    {
    	lastApClicked=a;
    }
    
    public void sendOptionsRequest(int ap_id,ActionPossibilityWidget apw)
    {
    	lastApClicked=apw;
    	myApp.requestOptions(ap_id);
    }
    
    public void handleOptionsResponse(String res, int id)
    {
    	lastApClicked.handleOptionsResponse(res,id);
    }
    
    public abstract void waitRunning();
    
    public abstract void sendRequest();
    
    public abstract void sendActionRequest(int id);
    
    //DISPLAY AND GRAPHIC UTILITIES
    
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
    
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
    	super.onActivityResult(requestCode, resultCode, data);
    	switch(requestCode){
        	case(SETTINGS_CODE):{
        		if(resultCode==Activity.RESULT_OK)
        		{
        			myPreferences.loadPreferences();
        			myApp.updatePreferences();
        			myApp.unsetSettings();
        			//myApp.setImagesRate(myPreferences.getImagesRate());
        			//Log.i("AccompanyGUI","onActivityResult");
        		}
        	} break;
    	}
    }
    
   //to close this activity
  	public void halt()
  	{
  		//this.setContentView(null);
  		this.finish();
  		this.onDestroy();
  	}
    
    /****************************/
    /* the options menu methods */
    /****************************/
    @Override
    public boolean onCreateOptionsMenu(final android.view.Menu menu) {
    	/*Log.e("menu","create opt");
		final MenuInflater inflater = this.getMenuInflater();
		inflater.inflate(R.menu.optmenu, menu);
		return (super.onCreateOptionsMenu(menu));*/
    	return false;
    }
    
    public void setPreferences(boolean speech_mode) {
	    SharedPreferences preferences = getSharedPreferences("accompany_gui_ros",
	        MODE_PRIVATE);
	    SharedPreferences.Editor editor = preferences.edit();

	    editor.putString("ros_master_ip",myPreferences.getRosMasterIP());
	    //Log.e("ACC","imr "+myPreferences.getImagesRate());
	    editor.putInt("images_rate", myPreferences.getImagesRate());
	    editor.putBoolean("speech_mode", speech_mode);
	   
	    editor.commit();
	  }
    
    public abstract void isMovingButton(boolean b);
    
    @Override
    public void onAttachedToWindow()
    {
    	//Log.e("Accompany-GUI-activity","on attached to window");
    	//this.getWindow().setType(WindowManager.LayoutParams.TYPE_KEYGUARD);
    	//Log.e("Accompany-GUI-activity","on attached to window 2");
    	super.onAttachedToWindow();
    	//Log.e("Accompany-GUI-activity","on attached to window 3");
    }
    
    @Override
    public void onPause()
    {
    	Log.i("Accompany-GUI-activity","on pause");
    	super.onPause();
    }

    @Override 
    public void onBackPressed()
    {
    	Log.i("Accompany-GUI-activity","on back pressed");
    	super.onBackPressed();
    }
    
}

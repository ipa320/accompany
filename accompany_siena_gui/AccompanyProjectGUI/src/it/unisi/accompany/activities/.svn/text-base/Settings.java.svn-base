package it.unisi.accompany.activities;

import it.unisi.accompany.AccompanyGUIApp;
import it.unisi.accompany.R;
import android.app.Activity;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.os.Bundle;
import android.util.Log;
import android.view.KeyEvent;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.widget.Button;
import android.widget.EditText;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.TextView;

public class Settings extends Activity {
	
	protected final String TAG = "AccompanyGUI-Settings";
	
	protected String RosMasterIP="10.0.1.5";
	//protected String DatabaseIp="10.0.1.5";
	protected String DatabasePort="http://10.0.1.5:9995/";
	protected String RobotPort="http://10.0.1.5:9996/";
	protected boolean speechMode=false;
	protected int cob_version;
	protected int exp_update=1;
	protected int ap_update=1;
	
	private EditText editIP;
	//private EditText editDbIp;
	private EditText editDbUrl;
	private EditText editRobotUrl;
	private RadioGroup cob_version_rg;
	private RadioButton cob_32_rb;
	private RadioButton cob_36_rb;
	private EditText editApFrq;  //AP/expression update frequencies
	private EditText editExpFrq;
	private Button save;
	
	protected AccompanyGUIApp myApp;
	
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
        setContentView(R.layout.settings);
        
        myApp=(AccompanyGUIApp)this.getApplication();
        
        editIP=(EditText)this.findViewById(R.id.master_et);
        //editDbIp=(EditText)this.findViewById(R.id.dbip_et);
        editDbUrl=(EditText)this.findViewById(R.id.db_port_et);
        editRobotUrl=(EditText)this.findViewById(R.id.robot_port_et);
        save=(Button)this.findViewById(R.id.ok_btn_settings);
        cob_32_rb=(RadioButton)this.findViewById(R.id.cob32_rb);
        cob_36_rb=(RadioButton)this.findViewById(R.id.cob36_rb);
        cob_version_rg=(RadioGroup)this.findViewById(R.id.cob_version_rg);
        
        editApFrq  = (EditText)this.findViewById(R.id.ap_update_et);
        editExpFrq = (EditText)this.findViewById(R.id.exp_update_et);
        
        readPreferences();
        
        Bundle b= getIntent().getExtras();
        
        if (b==null)
        {
        	editIP.setFocusable(false);
        	editIP.setBackgroundColor(Color.BLACK);
        	editIP.setTextColor(Color.WHITE);
        	editDbUrl.setOnEditorActionListener(new TextView.OnEditorActionListener() {
      			
      			@Override
      			public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
      				if(actionId==EditorInfo.IME_NULL && event.getAction()==KeyEvent.ACTION_DOWN)
      				{
      					readValues();
     					setPreferences();
     					
     					setResult(Activity.RESULT_OK,null);
     					finish();
      				}
      				return false;
      			}
      		});
        	editRobotUrl.setOnEditorActionListener(new TextView.OnEditorActionListener() {
      			
      			@Override
      			public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
      				if(actionId==EditorInfo.IME_NULL && event.getAction()==KeyEvent.ACTION_DOWN)
      				{
      					readValues();
     					setPreferences();
     					
     					setResult(Activity.RESULT_OK,null);
     					finish();
      				}
      				return false;
      			}
      		});
        	save.setOnClickListener(new View.OnClickListener(){
				@Override
				public void onClick(View v) {
					readValues();
					setPreferences();
					
					setResult(Activity.RESULT_OK,null);
					finish();
				}});
        	 
        }
        else
        {
        	editIP.setFocusable(true);
        	save.setOnClickListener(new View.OnClickListener(){
				@Override
				public void onClick(View v) {
					readValues();
					setPreferences();
					
					setResult(Activity.RESULT_OK,null);
					finish();
				}});
        	editIP.setOnEditorActionListener(new TextView.OnEditorActionListener() {
     			
     			@Override
     			public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
     				if(actionId==EditorInfo.IME_NULL && event.getAction()==KeyEvent.ACTION_DOWN)
     				{
     					RosMasterIP= editIP.getText().toString().replace("\\r", "").replace("\\n","");     //robot
    					DatabasePort=editDbUrl.getText().toString().replace("\\r", "").replace("\\n","");
    					RobotPort=editRobotUrl.getText().toString().replace("\\r", "").replace("\\n","");
    					setPreferences();
    					
    					setResult(Activity.RESULT_OK,null);
    					finish();
     				}
     				return false;
     			}
     		});
        	editDbUrl.setOnEditorActionListener(new TextView.OnEditorActionListener() {
     			
     			@Override
     			public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
     				if(actionId==EditorInfo.IME_NULL && event.getAction()==KeyEvent.ACTION_DOWN)
     				{
     					readValues();
    					setPreferences();
    					
    					setResult(Activity.RESULT_OK,null);
    					finish();
     				}
     				return false;
     			}
     		});
        	editRobotUrl.setOnEditorActionListener(new TextView.OnEditorActionListener() {
     			
     			@Override
     			public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
     				if(actionId==EditorInfo.IME_NULL && event.getAction()==KeyEvent.ACTION_DOWN)
     				{
     					readValues();
    					setPreferences();
    					
    					setResult(Activity.RESULT_OK,null);
    					finish();
     				}
     				return false;
     			}
     		});
        }
	}
	

	protected void readValues()
	{
		RosMasterIP= editIP.getText().toString().replace("\\r", "").replace("\\n","");     //robot
		DatabasePort=editDbUrl.getText().toString().replace("\\r", "").replace("\\n","");
		RobotPort=editRobotUrl.getText().toString().replace("\\r", "").replace("\\n","");
		ap_update= Integer.parseInt(editApFrq.getText().toString());
		exp_update= Integer.parseInt(editExpFrq.getText().toString());
	}
	
    //reading the current shared preferences:
    private void readPreferences() {
        SharedPreferences preferences = getSharedPreferences("accompany_gui_ros",
            MODE_PRIVATE);
        //reading the current shared preferences:
        RosMasterIP = preferences.getString("ros_master_ip", RosMasterIP);  //robot
        DatabasePort=preferences.getString("database_url", DatabasePort);
        speechMode=preferences.getBoolean("speech_mode", speechMode);
        RobotPort=preferences.getString("status_url", RobotPort);
        cob_version = preferences.getInt("cob_version",AccompanyGUIApp.COB36);
             
        ap_update= preferences.getInt("actionpossibilities_update_frequency",ap_update);
        exp_update= preferences.getInt("expression_update_frequency",exp_update);
        
        //writing the current preferences on screen:
        editIP.setText(RosMasterIP);
        editDbUrl.setText(DatabasePort);
        editRobotUrl.setText(RobotPort);
        editApFrq.setText(Integer.toString(ap_update));
        editExpFrq.setText(Integer.toString(exp_update));
        switch(cob_version)
        {
        	case AccompanyGUIApp.COB32:
        	{
        		cob_32_rb.setChecked(true);
        		cob_36_rb.setChecked(false);
        	} break;
        	case AccompanyGUIApp.COB36:
        	{
        		cob_36_rb.setChecked(true);
        		cob_32_rb.setChecked(false);
        	} break;
        	default:
        	{
        		cob_36_rb.setChecked(true);
        		cob_32_rb.setChecked(false);
        	} break;
        }
     }
         
    private void setPreferences() {
	    SharedPreferences preferences = getSharedPreferences("accompany_gui_ros",
	        MODE_PRIVATE);
	    SharedPreferences.Editor editor = preferences.edit();

	    editor.putString("ros_master_ip",RosMasterIP);
	    //editor.putInt("images_rate", images_rate);
	    editor.putString("database_url", DatabasePort);
	    editor.putString("status_url", RobotPort);
	    editor.putBoolean("speech_mode", speechMode);
	    editor.putInt("cob_version",cob_version);
	    
	    editor.putInt("actionpossibilities_update_frequency",ap_update);
	    editor.putInt("expression_update_frequency",exp_update);
	    //debug
	   /* Log.e("New lines","RosMaster ip:" + RosMasterIP +" - "+String.format("\\u%04X",RosMasterIP));
	    Log.e("New lines","Database url: "+DatabasePort+" - " + String.format("\\u%04X",DatabasePort));
	    Log.e("New lines","Robot url: "+ RobotPort+" - "+ String.format("\\u%04X",RobotPort));*/
	   /* Log.e("New lines","RosMaster: "+RosMasterIP);
	    Log.e("New lines","Db url: "+DatabasePort);
	    Log.e("New lines","Robot url: : "+RobotPort);*/
	    
	    editor.commit();
	  }
    
    public void onVersionChanged(View v)
    {
    	boolean checked = false;
    	try{
    		checked = ((RadioButton) v).isChecked();
    	}
    	catch (Exception e)
    	{
    		Log.e(TAG,"Error in reading cob version...");
    		return;
    	}
    	switch(v.getId())
    	{
	    	case R.id.cob32_rb:
	    	{
	    		if(checked)
	    			cob_version=AccompanyGUIApp.COB32;
	    	}break;
	    	case R.id.cob36_rb:
	    	{
	    		if(checked)
	    			cob_version=AccompanyGUIApp.COB36;
	    	} break;
    	}
    }
}
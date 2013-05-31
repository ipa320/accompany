package it.unisi.accompany.activities;

import it.unisi.accompany.AccompanyGUIApp;
import it.unisi.accompany.R;
import android.app.Activity;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.view.KeyEvent;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

public class Settings extends Activity {
	
	protected String RosMasterIP="10.0.1.5";
	protected String DatabaseIp="10.0.1.5";
	protected String DatabasePort="9995";
	protected String RobotPort="9996";
	protected boolean speechMode=true;
	
	private EditText editIP;
	private EditText editDbIp;
	private EditText editDbPort;
	private EditText editRobotPort;
	private Button save;
	
	protected AccompanyGUIApp myApp;
	
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
        setContentView(R.layout.settings);
        
        myApp=(AccompanyGUIApp)this.getApplication();
        
        editIP=(EditText)this.findViewById(R.id.master_et);
        editDbIp=(EditText)this.findViewById(R.id.dbip_et);
        editDbPort=(EditText)this.findViewById(R.id.db_port_et);
        editRobotPort=(EditText)this.findViewById(R.id.robot_port_et);
        save=(Button)this.findViewById(R.id.ok_btn_settings);
        
        readPreferences();
        
        Bundle b= getIntent().getExtras();
        
        if (b==null)
        {
        	editIP.setFocusable(false);
        	editDbIp.requestFocus();
        	save.setOnClickListener(new View.OnClickListener(){
				@Override
				public void onClick(View v) {
					RosMasterIP= editIP.getText().toString();     //robot
					DatabaseIp= editDbIp.getText().toString();
					DatabasePort=editDbPort.getText().toString();
					RobotPort=editRobotPort.getText().toString();
					setPreferences();
					
					setResult(Activity.RESULT_OK,null);
					finish();
				}});
        	 editDbIp.setOnEditorActionListener(new TextView.OnEditorActionListener() {
     			
     			@Override
     			public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
     				if(actionId==EditorInfo.IME_NULL && event.getAction()==KeyEvent.ACTION_DOWN)
     				{
     					RosMasterIP= editIP.getText().toString();     //robot
    					DatabaseIp= editDbIp.getText().toString();
    					DatabasePort=editDbPort.getText().toString();
    					RobotPort=editRobotPort.getText().toString();
    					setPreferences();
    					
    					setResult(Activity.RESULT_OK,null);
    					finish();
     				}
     				return false;
     			}
     		});
        	editDbPort.setOnEditorActionListener(new TextView.OnEditorActionListener() {
      			
      			@Override
      			public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
      				if(actionId==EditorInfo.IME_NULL && event.getAction()==KeyEvent.ACTION_DOWN)
      				{
      					RosMasterIP= editIP.getText().toString();     //robot
     					DatabaseIp= editDbIp.getText().toString();
     					DatabasePort=editDbPort.getText().toString();
     					RobotPort=editRobotPort.getText().toString();
     					setPreferences();
     					
     					setResult(Activity.RESULT_OK,null);
     					finish();
      				}
      				return false;
      			}
      		});
        	editRobotPort.setOnEditorActionListener(new TextView.OnEditorActionListener() {
      			
      			@Override
      			public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
      				if(actionId==EditorInfo.IME_NULL && event.getAction()==KeyEvent.ACTION_DOWN)
      				{
      					RosMasterIP= editIP.getText().toString();     //robot
     					DatabaseIp= editDbIp.getText().toString();
     					DatabasePort=editDbPort.getText().toString();
     					RobotPort=editRobotPort.getText().toString();
     					setPreferences();
     					
     					setResult(Activity.RESULT_OK,null);
     					finish();
      				}
      				return false;
      			}
      		});
        	 
        }
        else
        {
        	save.setOnClickListener(new View.OnClickListener(){
				@Override
				public void onClick(View v) {
					RosMasterIP= editIP.getText().toString();     //robot
					DatabaseIp= editDbIp.getText().toString();
					DatabasePort=editDbPort.getText().toString();
					RobotPort=editRobotPort.getText().toString();
					setPreferences();
					
					setResult(Activity.RESULT_OK,null);
					finish();
				}});
        	editDbIp.setOnEditorActionListener(new TextView.OnEditorActionListener() {
     			
     			@Override
     			public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
     				if(actionId==EditorInfo.IME_NULL && event.getAction()==KeyEvent.ACTION_DOWN)
     				{
     					RosMasterIP= editIP.getText().toString();     //robot
    					DatabaseIp= editDbIp.getText().toString();
    					DatabasePort=editDbPort.getText().toString();
    					RobotPort=editRobotPort.getText().toString();
    					setPreferences();
    					
    					setResult(Activity.RESULT_OK,null);
    					finish();
     				}
     				return false;
     			}
     		});
        	editIP.setOnEditorActionListener(new TextView.OnEditorActionListener() {
     			
     			@Override
     			public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
     				if(actionId==EditorInfo.IME_NULL && event.getAction()==KeyEvent.ACTION_DOWN)
     				{
     					RosMasterIP= editIP.getText().toString();     //robot
    					DatabaseIp= editDbIp.getText().toString();
    					DatabasePort=editDbPort.getText().toString();
    					RobotPort=editRobotPort.getText().toString();
    					setPreferences();
    					
    					setResult(Activity.RESULT_OK,null);
    					finish();
     				}
     				return false;
     			}
     		});
        	editDbPort.setOnEditorActionListener(new TextView.OnEditorActionListener() {
     			
     			@Override
     			public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
     				if(actionId==EditorInfo.IME_NULL && event.getAction()==KeyEvent.ACTION_DOWN)
     				{
     					RosMasterIP= editIP.getText().toString();     //robot
    					DatabaseIp= editDbIp.getText().toString();
    					DatabasePort=editDbPort.getText().toString();
    					RobotPort=editRobotPort.getText().toString();
    					setPreferences();
    					
    					setResult(Activity.RESULT_OK,null);
    					finish();
     				}
     				return false;
     			}
     		});
        	editRobotPort.setOnEditorActionListener(new TextView.OnEditorActionListener() {
     			
     			@Override
     			public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
     				if(actionId==EditorInfo.IME_NULL && event.getAction()==KeyEvent.ACTION_DOWN)
     				{
     					RosMasterIP= editIP.getText().toString();     //robot
    					DatabaseIp= editDbIp.getText().toString();
    					DatabasePort=editDbPort.getText().toString();
    					RobotPort=editRobotPort.getText().toString();
    					setPreferences();
    					
    					setResult(Activity.RESULT_OK,null);
    					finish();
     				}
     				return false;
     			}
     		});
        }
	}
	

    //reading the current shared preferences:
    private void readPreferences() {
        SharedPreferences preferences = getSharedPreferences("accompany_gui_ros",
            MODE_PRIVATE);
        //reading the current shared preferences:
        RosMasterIP = preferences.getString("ros_master_ip", RosMasterIP);  //robot
        DatabaseIp = preferences.getString("database_ip", DatabaseIp);
        DatabasePort=preferences.getString("database_port", DatabasePort);
        speechMode=preferences.getBoolean("speech_mode", speechMode);
        RobotPort=preferences.getString("status_port", RobotPort);
             
        //writing the current preferences on screen:
        editIP.setText(RosMasterIP);
        editDbIp.setText(DatabaseIp);
        editDbPort.setText(DatabasePort);
        editRobotPort.setText(RobotPort);
     }
         
    private void setPreferences() {
	    SharedPreferences preferences = getSharedPreferences("accompany_gui_ros",
	        MODE_PRIVATE);
	    SharedPreferences.Editor editor = preferences.edit();

	    editor.putString("ros_master_ip",RosMasterIP);
	    //editor.putInt("images_rate", images_rate);
	    editor.putString("database_ip",DatabaseIp);
	    editor.putString("database_port", DatabasePort);
	    editor.putString("status_port", RobotPort);
	    editor.putBoolean("speech_mode", speechMode);
	    
	    editor.commit();
	  }

}
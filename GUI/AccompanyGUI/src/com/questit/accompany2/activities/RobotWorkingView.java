package com.questit.accompany2.activities;

import java.net.URI;

import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.MyRosActivity;
import org.ros.android.view.RosImageViewReversed;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import com.questit.accompany2.AccompanyGUI_app2;
import com.questit.accompany2.R;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.Matrix;
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
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.PopupWindow;
import android.widget.TextView;
import android.widget.FrameLayout.LayoutParams;

public class RobotWorkingView extends Activity {
	
	
	protected AccompanyGUI_app2 myApp;
	protected RobotWorkingView me;
	
	protected AbsoluteLayout my_layout;
	protected ImageView image;
	//protected ImageView mask;
	
	protected Matrix my_matrix;
	
	protected PopupWindow popupWindow;
	
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		//standrd things
		super.onCreate(savedInstanceState);
		this.setContentView(R.layout.robot_working_view);
		
		//recovering the application that owns this activity
		//and setting the activity variable
		myApp=(AccompanyGUI_app2)this.getApplication();
		myApp.setRobotWorkingView(this);
		me=this;
		
		//bring camera to front
		myApp.head_controller.bringCameraToFront();
		myApp.torso_controller.bringHome();
		
		 //Setting up the policies for the use of threads
        if (android.os.Build.VERSION.SDK_INT > 9) {
			StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder()
					.permitAll().build();
			StrictMode.setThreadPolicy(policy);
		}
		
		image=(ImageView)this.findViewById(R.id.robot_working_image_view);
		image.setImageBitmap(myApp.getLastImage());
		
		Log.i("AccompanyGUI","on create robotWorkingView");
		myApp.setRunningActivity(myApp.EXECUTE_VIEW);
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
	//to switch to the user view we need to launch the correct activity
	public void switchToUserView()
	{
		myApp.stopSubscribing();
		Log.i("ciao","cioa");
		final Intent intent = new Intent().setClass(RobotWorkingView.this.me, UserView.class);
		RobotWorkingView.this.startActivity(intent);
		finish();
	}
	
	//Switching to robot view (on banners clicks), the robot view activity is launched:
	public void switchToRobotView()
	{
		final Intent intent = new Intent().setClass(RobotWorkingView.this.me, RobotView.class);
		RobotWorkingView.this.startActivity(intent);
		finish();
	}
	
	//Switching to robot view (on banners clicks), the robot view activity is launched:
	public void switchToActionsList()
	{
		myApp.stopSubscribing();
		final Intent intent = new Intent().setClass(RobotWorkingView.this.me, ActionsListView.class);
		RobotWorkingView.this.startActivity(intent);
		finish();
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
			//do nothing (in this view the switch to the list is not enabled)
		}
		return true;
		case R.id.settings: {
			//do nothing (in this view settings are not enabled)
		}
		return true;
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
    
    /****************************/
    
	//to close this activity
	public void halt()
	{
		this.finish();
	}

	/*@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {
		try{
		NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),
			    new URI("http://192.168.1.109:11311/"));
			nodeMainExecutor.execute(image, nodeConfiguration.setNodeName("AccompanyGUI/WorkingView_image"));
		}catch(Exception e){
			Log.i("AccompanyGUI","Working View error on creating image subscriber!");
		}
	}*/
}

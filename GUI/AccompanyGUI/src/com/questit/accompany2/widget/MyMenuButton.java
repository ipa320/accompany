package com.questit.accompany2.widget;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.StringTokenizer;

import com.questit.accompany2.activities.ActionsListView;

import android.content.Context;
import android.graphics.Color;
import android.util.Log;
import android.view.View;
import android.widget.FrameLayout;
import android.widget.TextView;

public class MyMenuButton extends FrameLayout{

	protected final int TEXTSIZE=20;
	
	protected String text;
	protected String command;
	protected String phrase;
	protected String original_phrase;
	protected ActionsListView act;
	protected TextView tv;
	
	
	protected ActionFatherButton parent;
	protected MyPopupMenu menu;
	
	public MyMenuButton(Context context, String t, String c,String phr, ActionsListView a,
			ActionFatherButton p,MyPopupMenu men) {
		super(context);
		text=t;
		tv=new TextView(context);
		tv.setTextColor(Color.WHITE);
		tv.setTextSize(TEXTSIZE);
		tv.setBackgroundColor(Color.TRANSPARENT);
		tv.setText(text);
		this.addView(tv);
		this.setPadding(5, 0, 5, 5);
		command=c;
		original_phrase=phrase=phr;
		//working on the phrase, to properly encode it:
		phrase="";
		StringTokenizer st= new StringTokenizer(original_phrase," ");
		phrase=st.nextToken();
		while (st.hasMoreTokens())
		{
			phrase=phrase+"_"+st.nextToken();
		}
		act=a;
		parent=p;
		menu=men;
		this.setOnClickListener(new View.OnClickListener()
		    	{
					
			@Override
			public void onClick(View v) {
				if (original_phrase.equals("athccendo lootchee")) original_phrase="accendo luci";
    			if (original_phrase.equals("spengo lootchee")) original_phrase="spengo luci";
    					
    			act.toastMessage(original_phrase);
    			act.sendActionListActionRequest(command, phrase);
    			
    			parent.reset();
				menu.dismiss();
			}
			
					/*@Override
					public void onClick(View v) {
						Log.e("Robot","Sending: execute "+command+" "+phrase);
		    			//Connecting to robot server
		    			try 
		    			{
		    				Log.i("INFO","socket da creare");
		    				COB_sock=new Socket(robotIp,robot_port);
		    				//COB_sock=new Socket("192.168.1.104",CareOBotPort);
		    				Log.i("INFO","socket creata");
		    				//sending the command to the robot (remote command server)
		    				if (COB_sock!=null)
		    				{
		    					PrintWriter os = new PrintWriter(COB_sock.getOutputStream());
		    					//os.write("fwd 1.0 5");
		    					os.write("execute " + command+" "+phrase);
		    					//os.write(this.command);
		    					os.flush();
		    					os.close();
		    					COB_sock.close();
		    					COB_sock=null;
		    					
		    					if (original_phrase.equals("athccendo lootchee")) original_phrase="accendo luci";
		    					if (original_phrase.equals("spengo lootchee")) original_phrase="spengo luci";
		    					
		    					act.showRobotExecutingCommandView(phrase);
		    					act.toastMessage(original_phrase);
		    				}
		    			} catch (UnknownHostException e) {
		    				Log.e("ERR","Unknown host for the Care-o-bot");
		    			//toastMessage("Please set the correct Ip and port for the care-o.bot");
		    			} catch (IOException e) {
		    				Log.e("ERR","IO Exception for the Care-o-bot");		
		    			}
		    			parent.reset();
						menu.dismiss();
					}*/
				});
		// TODO Auto-generated constructor stub
	}	
	
	public void setTextColor(int c)
	{
		tv.setTextColor(c);
	}
}

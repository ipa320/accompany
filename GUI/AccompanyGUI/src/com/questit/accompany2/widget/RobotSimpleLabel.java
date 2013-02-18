package com.questit.accompany2.widget;


import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.StringTokenizer;

import com.questit.accompany2.R;
import com.questit.accompany2.activities.RobotView;

import android.content.Context;
import android.graphics.Color;
import android.util.Log;
import android.view.View;
import android.widget.AbsoluteLayout;
import android.widget.Button;

public class RobotSimpleLabel  extends Button implements View.OnClickListener{

	protected final int MYCOLOR = Color.BLACK;
	//protected final int MYTEXTSIZE = 24;

	// default MAX & min values for width and height of the buttons.
	protected final int MAXHEIGHT=75;
	protected final int MAXWIDTH=300;
	protected final int MINHEIGHT=35;
	protected final int MINWIDTH=150;
	protected final int MINTEXTSIZE=20;
	protected final int MAXTEXTSIZE=30;
	
	protected String text;
	protected String command;
	protected String phrase;
	protected String original_phrase;
	protected double likelihood;
	protected int x;
	protected int y;
	protected int myHeight;
	protected int myWidth;
	
	protected RobotView act;
	protected AbsoluteLayout my_layout;
	
	
	@SuppressWarnings("deprecation")
	public RobotSimpleLabel(Context context,String name,String command,String p,double lik,int pos_x,int pos_y,
			AbsoluteLayout lay,RobotView a) {
		super(context);
		
		this.text=name;
		this.setText(text);
		//this.setTypeface(null,Typeface.BOLD);
		
		this.setTextColor(MYCOLOR);
		//this.setTextSize(MYTEXTSIZE);
		//this.setBackgroundColor(Color.GREEN);
		//this.setBackgroundColor(Color.TRANSPARENT);
		this.setBackgroundDrawable(getResources().getDrawable(R.drawable.my_shape));
		this.setPadding(15, 5, 15, 5);
		this.setOnClickListener(this);

		this.x=pos_x;
		this.y=pos_y;
		
		this.command=command;
		this.original_phrase=this.phrase=p;
		this.likelihood=lik;
		
		this.act=a;
		this.my_layout=lay;
		
		myHeight=(MINHEIGHT+(int)((MAXHEIGHT-MINHEIGHT)*likelihood));
		myWidth=(MINWIDTH+(int)((MAXWIDTH-MINWIDTH)*likelihood));
		this.setWidth(myWidth);
		this.setHeight(myHeight);
		this.setTextSize((float)(MINTEXTSIZE+(MAXTEXTSIZE-MINTEXTSIZE)*likelihood));
		
		//working on the phrase, to properly encode it:
		phrase="";
		StringTokenizer st= new StringTokenizer(original_phrase," ");
		phrase=st.nextToken();
		while (st.hasMoreTokens())
		{
			phrase=phrase+"_"+st.nextToken();
		}
	}

	public void onClick(View v) {

		if (original_phrase.contains("athccendo loothcee")) original_phrase="accendo luci";
		if (original_phrase.contains("spengo loothcee")) original_phrase="spengo luci";
				
		//setting the robot state as busy aand removing all labels temporarly
		act.toastMessage(original_phrase);
		act.sendRobotActionRequest(command, phrase);
	}
	
	/*@Override
	public void onClick(View v) {
		//toast message with the command to be executed (debug)
		//act.toastMessage("executing :"+command);
		Log.e("Robot","Sending: execute "+ command+" "+phrase);
		//Connecting to robot server
		try 
		{
			Log.i("INFO","socket da creare");
			COB_sock=new Socket(CareOBotIP,CareOBotPort);
			//COB_sock=new Socket("192.168.1.104",CareOBotPort);
			Log.i("INFO","socket creata");
			//sending the command to the robot (remote command server)
			if (COB_sock!=null)
			{
				PrintWriter os = new PrintWriter(COB_sock.getOutputStream());
				//os.write("fwd 1.0 5");
				os.write("execute " + command +" "+phrase);
				//os.write(this.command);
				os.flush();
				os.close();
				COB_sock.close();
				COB_sock=null;
				
				if (original_phrase.contains("athccendo loothcee")) original_phrase="accendo luci";
				if (original_phrase.contains("spengo loothcee")) original_phrase="spengo luci";
				
				//setting the robot state as busy aand removing all labels temporarly
				act.showRobotExecutingCommandView(phrase);
				act.toastMessage(original_phrase);
			}
		} catch (UnknownHostException e) {
			Log.e("ERR","Unknown host for the Care-o-bot");
		//toastMessage("Please set the correct Ip and port for the care-o.bot");
		} catch (IOException e) {
			Log.e("ERR","IO Exception for the Care-o-bot");
		}
		
		//removing the view (the condition will change and a new list of ap will be loaded
		//with maybe new options after this action's execution)
		//my_layout.removeView(this);
	}*/
	
	
	
	public void setPosition(int xx, int yy)
	{
		this.x=xx;
		this.y=yy;
	}
	
	public int getMyX()
	{
		return this.x;
	}
	
	public int getMyY()
	{
		return this.y;
	}
	
	public AbsoluteLayout.LayoutParams randomPose(int sizeX, int sizeY)
	{
		
		sizeX=Math.max(sizeX-200,0);
		sizeY=Math.max(sizeY-100,0);
		double r1=Math.random();
		double r2=Math.random();
		x=50+(int)(r1*sizeX);
		y=50+(int)(r2*sizeY);
		
		AbsoluteLayout.LayoutParams p=new AbsoluteLayout.LayoutParams(AbsoluteLayout.LayoutParams.WRAP_CONTENT, AbsoluteLayout.LayoutParams.WRAP_CONTENT, x,y);
		return p;
	}

	public AbsoluteLayout.LayoutParams getParams()
	{		
		AbsoluteLayout.LayoutParams p=new AbsoluteLayout.LayoutParams(AbsoluteLayout.LayoutParams.WRAP_CONTENT, AbsoluteLayout.LayoutParams.WRAP_CONTENT, x,y);
		return p;
	}
	
	public void autoSetParams(int w)
	{
		AbsoluteLayout.LayoutParams p=getParams();
		p.width=w;
		this.setLayoutParams(p);
	}
	
	public void autoSetParams()
	{
		AbsoluteLayout.LayoutParams p=getParams();
		this.setLayoutParams(p);
	}

}

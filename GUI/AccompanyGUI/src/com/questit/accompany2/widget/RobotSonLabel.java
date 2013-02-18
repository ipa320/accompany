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

public class RobotSonLabel extends Button implements View.OnClickListener
{
	protected final int MYCOLOR=Color.BLACK;
	protected final int MYTEXTSIZE=22;
	
	protected String name;
	protected int x;
	protected int y;
	protected RobotFatherLabel parent;
	
	protected RobotView act;
	
	protected String command;
	protected String phrase;
	protected String original_phrase;
	
	
	
	public RobotSonLabel(Context context,String Name, String command,String p, int plus_x,int plus_y, RobotFatherLabel pparent,
			RobotView a)
	{
		super(context);
		this.name=Name;
		this.setText(name);
		this.setTextColor(Color.BLACK);
		this.setTextSize(MYTEXTSIZE);
		//this.setTypeface(null,Typeface.BOLD);
		//this.setBackgroundColor(Color.TRANSPARENT);
		//this.setBackgroundColor(Color.GREEN);
		//this.setBackgroundDrawable(getResources().getDrawable(R.drawable.label_bck));
		//this.setBackgroundDrawable(getResources().getDrawable(R.drawable.cloud_son_button_shape));
		this.setBackgroundDrawable(getResources().getDrawable(R.drawable.cloud_son_button_shape));
		this.setOnClickListener(this);
		this.setPadding(12, 4, 12, 4);
		this.parent=pparent;
		this.x=parent.getPosX()+plus_x;
		this.y=parent.getPosY()+plus_y;
		this.command=command;
		this.phrase=p;
		
		this.act=a;
		
		//this.setTextColor(Color.BLUE);
		this.setTextColor(MYCOLOR);
		
		//working on the phrase, to properly encode it:
		original_phrase=phrase;
		phrase="";
		StringTokenizer st= new StringTokenizer(original_phrase," ");
		phrase=st.nextToken();
		while (st.hasMoreTokens())
		{
			phrase=phrase+"_"+st.nextToken();
		}
	}
	
	@Override
	public void onClick(View v) {
				
		if (original_phrase.equals("athccendo lootchee")) original_phrase="accendo luci";
		if (original_phrase.equals("spengo lootchee")) original_phrase="spengo luci";
				
		//calling the coreect procedure to set the robot busy
		act.toastMessage(original_phrase);
		act.sendRobotActionRequest(command, phrase);
		
		//call the method to return the layout in the original state--> reset the parent!
		parent.resetSons();
	}
	
	public int getMyX()
	{
		return this.x;
	}
	
	public int getMyY()
	{
		return this.y;
	}
	
	public void setPosition(int xx, int yy)
	{
		this.x=xx;
		this.y=yy;
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

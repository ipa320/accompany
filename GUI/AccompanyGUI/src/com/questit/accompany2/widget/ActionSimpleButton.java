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
import android.widget.FrameLayout.LayoutParams;

public class ActionSimpleButton extends FrameLayout{

	protected final int TEXTSIZE=20;	
	
	protected TextView tv;
	protected ActionsListView act;
	protected String command;
	protected String phrase;
	protected String original_phrase;
	
	protected View.OnClickListener myListener;
	
	public ActionSimpleButton(Context context,String name,String cmd,String p,
			ActionsListView ac) 
	{
		super(context);
		command=cmd;
		this.original_phrase=this.phrase=p;
		this.setBackgroundColor(Color.TRANSPARENT);
		this.setClickable(true);
		tv= new TextView(context);
		tv.setTextColor(Color.WHITE);
		tv.setTextSize(TEXTSIZE);
		tv.setPadding(2,4,2,4);
		//this.setBackgroundColor(Color.TRANSPARENT);
		//this.setBackgroundDrawable(getResources().getDrawable(R.drawable.my_gradient_button));
		tv.setBackgroundColor(Color.TRANSPARENT);
		tv.setText(name);
		
		tv.setLayoutParams(new LayoutParams(LayoutParams.WRAP_CONTENT,LayoutParams.WRAP_CONTENT));
		this.addView(tv);
		
		this.act=ac;
		
		//working on the phrase, to properly encode it:
		phrase="";
		StringTokenizer st= new StringTokenizer(original_phrase," ");
		phrase=st.nextToken();
		while (st.hasMoreTokens())
		{
			phrase=phrase+"_"+st.nextToken();
		}
		
		myListener= new View.OnClickListener() {
			
			@Override
			public void onClick(View v) {
				if (original_phrase.contains("athccendo loothcee")) original_phrase="accendo luci";
				if (original_phrase.contains("spengo loothcee")) original_phrase="spengo luci";
				
				act.toastMessage(original_phrase);
				act.sendActionListActionRequest(command, phrase);
			}
		};
		
		this.setOnClickListener(myListener);
	}

}


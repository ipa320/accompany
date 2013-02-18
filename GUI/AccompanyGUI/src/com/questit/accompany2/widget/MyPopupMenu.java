package com.questit.accompany2.widget;


import java.util.ArrayList;

import com.questit.accompany2.activities.ActionsListView;
import com.questit.accompany2.data.Action;

import android.content.Context;
import android.graphics.Color;
import android.graphics.drawable.Drawable;
import android.widget.LinearLayout;
import android.widget.PopupWindow;
import android.widget.FrameLayout.LayoutParams;
import android.widget.PopupWindow.OnDismissListener;

public class MyPopupMenu extends PopupWindow{
	
	MyPopupMenu me;
	ActionsListView act;
	ActionFatherButton fab;

	public MyPopupMenu(Context context,LinearLayout contentView, int Width, int Height,
			ArrayList<Action> sons,Drawable d, ActionsListView a,
			ActionFatherButton f)
	{
		super(contentView,Width,Height);
		me=this;
		act=a;
		fab=f;
		for (int i=0; i<sons.size();i++)
	    {
	    	MyMenuButton b=new MyMenuButton(context,sons.get(i).label,
	    			sons.get(i).command,sons.get(i).phrase,act,f,this);
	    	b.setBackgroundColor(Color.TRANSPARENT);
	    	b.setTextColor(Color.WHITE);
	    	
	    	
	    	
	    	b.setLayoutParams(new LayoutParams(LayoutParams.FILL_PARENT,LayoutParams.WRAP_CONTENT));
	    	contentView.addView(b);
	    }
		
		this.setOutsideTouchable(true);
		this.setOnDismissListener(new OnDismissListener()
		{

			@Override
			public void onDismiss() {
				fab.reset();
			}
			
		});
    	this.setBackgroundDrawable(d);
	}
	
}


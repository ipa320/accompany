package com.questit.accompany2.widget;

import android.content.Context;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.widget.ImageView;

public class myImageView extends ImageView{
protected GestureDetector g;
	
	public myImageView(Context context) {
		super(context);
	}
	
	public myImageView(Context context,GestureDetector gd) {
		super(context);
		g=gd;
	}
	
	@Override
	public boolean onTouchEvent(MotionEvent event)
	{
		if (g!=null)
			if (g.onTouchEvent(event))
				return true;
			else
				return false;
		else
			return false;  //?
	}
	
	public void setGestureDetector(GestureDetector gd)
	{
		g=gd;
	}
	
}

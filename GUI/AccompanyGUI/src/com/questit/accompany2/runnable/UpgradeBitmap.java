package com.questit.accompany2.runnable;

import com.questit.accompany2.AccompanyGUI_app2;

import android.graphics.Bitmap;

public class UpgradeBitmap implements Runnable{

	protected AccompanyGUI_app2 app;
	protected Bitmap b;
	
	public UpgradeBitmap(Bitmap bm,AccompanyGUI_app2 a)
	{
		b=bm;
		app=a;
	}
	
	@Override
	public void run() {
		app.setBitmap(b);
	}

}

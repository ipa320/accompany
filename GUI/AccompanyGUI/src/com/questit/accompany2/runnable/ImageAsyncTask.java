package com.questit.accompany2.runnable;

import org.jboss.netty.buffer.ChannelBuffer;
import android.util.Log;

import com.questit.accompany2.AccompanyGUI_app2;

import sensor_msgs.CompressedImage;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Matrix;
import android.os.AsyncTask;

public class ImageAsyncTask extends AsyncTask<sensor_msgs.CompressedImage,Void,Bitmap>{

	protected AccompanyGUI_app2 myApp;
	
	protected BitmapFactory.Options options_bounds;
	boolean rotation=false;
	Matrix m;
	
	public ImageAsyncTask(AccompanyGUI_app2 my)
	{
		super();
		myApp = my;
		m=new Matrix();
		m.reset();
		m.postRotate(180);
	}
	
	public void setRotation(boolean b)
	{
		rotation=b;
	}
	
	@Override
	protected Bitmap doInBackground(CompressedImage... params) {
		for (CompressedImage par : params)
		{
			   options_bounds= new BitmapFactory.Options();
			   //options_bounds.inJustDecodeBounds= true;
			   options_bounds.inPurgeable=true;
			   Log.i("AccompanyGUI-images","Images Async task");
			   //options_bounds.inScaled=false;
			   //options_bounds.inInputShareable=true;
			   ChannelBuffer buffer = par.getData();
			   byte[] data = buffer.array();
			   options_bounds.inJustDecodeBounds=false;
			   //Matrix m= new Matrix();
			   //if (!rotation) m.setRotate(180);
			   try{
				   Bitmap b = BitmapFactory.decodeByteArray(data, buffer.arrayOffset(), buffer.readableBytes(),options_bounds);
				   if (!rotation) 
					   {//b=Bitmap.createBitmap(b, 0, 0, b.getWidth(), b.getHeight(), m, false);
					   //Canvas c= new Canvas(b);
					   // c.drawBitmap(b, m, null);
					   	}
				   //b=Bitmap.createBitmap(b, 0, 0, width, height, m, false);
				   //b=Bitmap.createBitmap(b, 0, 0, b.getWidth(), b.getHeight(), m, false);
				   //Bitmap bb= Bitmap.createBitmap(b,0,0,b.getWidth(),b.getHeight(),m,false);
				   //b.recycle();
				   System.gc();
				   return b;
			   }catch(Exception e){Log.e("AccompanyGUI-image","Wrong image decode");}
		}
		return null;
	}

	@Override
	protected void onPostExecute(Bitmap b)
	{
		myApp.setBitmap2(b,m);
	}
	
}
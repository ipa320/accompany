package com.questit.accompany2.RosNodes;

import org.ros.android.AccompanyBitmapFromCompressedImage;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.MessageCallable;
import org.ros.exception.RosRuntimeException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import com.questit.accompany2.AccompanyGUI_app2;
import com.questit.accompany2.runnable.ImageAsyncTask;

import android.graphics.Bitmap;
import android.os.Handler;
import android.util.Log;

public class ImagesSubscriber<T> implements NodeMain{

	private String topicName;
	private String messageType;
	//private MessageCallable<Bitmap, T> callable;
	private AccompanyBitmapFromCompressedImage callable;
	private Handler handler;
	protected AccompanyGUI_app2 myApp;
	public int message_sampling_rate=10;
	
	protected ConnectedNode cn;
	protected Subscriber<sensor_msgs.CompressedImage> subscriber;
	
	//count to avoid the block of the device due to too much bitmaps to be processed
	int num=0;
	
	public ImagesSubscriber(Handler h,AccompanyGUI_app2 ma, int rate)
	{
		super();
		myApp=ma;
		handler=h;
		message_sampling_rate=rate;
	}
	
	public void setRate(int rate)
	{
		message_sampling_rate=rate;
	}
	
	@Override
	public void onError(Node arg0, Throwable arg1) {
		Log.e("AccompanyGUI","Error: Image subscriber error!");
	}

	@Override
	public void onShutdown(Node arg0) {
		if (subscriber!=null) subscriber.shutdown();
		Log.e("Accompany-Ros","shtdown images subscriber...");
	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		cn=connectedNode;
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("GuiImageSubscriber");
	}

	  public void setTopicName(String topicName) {
		  this.topicName = topicName;
	  }

	  public void setMessageType(String messageType) {
		  this.messageType = messageType;
	  }

	  //public void setMessageToBitmapCallable(MessageCallable<Bitmap, T> callable) {
	  public void setMessageToBitmapCallable(AccompanyBitmapFromCompressedImage callable) {
	    this.callable = callable;
	  }
	  
	  public void startSubscribing()
	  {
		  try{
		  subscriber = cn.newSubscriber(topicName, messageType);
			num=0;
		    subscriber.addMessageListener(new MessageListener<sensor_msgs.CompressedImage>() {
		      @Override
		      public void onNewMessage(final sensor_msgs.CompressedImage message) { 
		    	  Log.e("AccompanyGUI-images","new image message");
		    	  if (num%message_sampling_rate==0)
		    	  {
		    		  handler.post(new Runnable() {
		    			  @Override
		    			  public void run() {
		    				  Log.i("AccomapanyGUI-images","image received, decoding...");
		    				  //callable.setRotation(myApp.getImageRotation());
		    				  //myApp.setBitmap(callable.call(message));
		    				  ImageAsyncTask iat= new ImageAsyncTask(myApp);
				    		  iat.setRotation(myApp.getImageRotation());
				    		  iat.execute(message);
				    		  num=0;
		    			  }
		    		  });
		    		  num=0;
		    	  }
		    	  num++;
		      }
		    });
		  }catch(Exception e)
		  {
			  Log.e("AccompanyGUI-images","exception in subscribing images");
			  if (subscriber!=null) subscriber.shutdown();
			  if (cn!=null) cn.shutdown();
			  throw new RosRuntimeException(e);
		  }
	  }
	
	  public void stopSubscribing()
	  {
		  subscriber.shutdown();
	  }
	  
	  public int getMyPid()
	  {
		  return android.os.Process.myPid();
	  }
}

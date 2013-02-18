package com.questit.accompany2.threads;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.Socket;
import java.net.UnknownHostException;
import java.sql.Timestamp;

import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import com.questit.accompany2.AccompanyGUI_app2;
import com.questit.accompany2.activities.RobotView;
import com.questit.accompany2.runnable.UpgradeBitmap;
import com.questit.accompany2.widget.myImageView;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Matrix;
import android.os.Environment;
import android.os.Handler;
import android.util.Log;
import android.view.GestureDetector;
import android.widget.FrameLayout;

public class camClient implements NodeMain{
	
	protected Subscriber<sensor_msgs.CompressedImage> mySub;
	
	@Override
	public void onError(Node arg0, Throwable arg1) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onShutdown(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onStart(ConnectedNode arg0) {
		// TODO Auto-generated method stub
		
	}
	

	  @Override
	  public GraphName getDefaultNodeName() {
		  return GraphName.of("camera_client");
	  }

	

}

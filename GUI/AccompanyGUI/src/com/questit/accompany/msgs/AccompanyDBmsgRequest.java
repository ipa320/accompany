package com.questit.accompany.msgs;

public abstract interface AccompanyDBmsgRequest extends org.ros.internal.message.Message{
	
	 // Field descriptor #5 Ljava/lang/String;
	  public static final java.lang.String _TYPE = "com/questit/accompany/msgs/AccompanyDBmsgRequest";
	  
	  // Field descriptor #5 Ljava/lang/String;
	  public static final java.lang.String _DEFINITION = "int64 request\nstring param\nint64 sonReq\n";
	  
	  // Method descriptor #11 ()J
	  public abstract long getRequest();
	  
	  // Method descriptor #13 (J)V
	  public abstract void setRequest(long arg0);
	  
	  // Method descriptor #11 ()J
	  public abstract String getParam();
	  
	  // Method descriptor #13 (J)V
	  public abstract void setParam(String arg0);
	  
	  // Method descriptor #11 ()J
	  public abstract long getSonReq();
	  
	  // Method descriptor #13 (J)V
	  public abstract void setSonReq(long arg0);

}

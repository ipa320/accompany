package com.questit.accompany.msgs;

public abstract interface AccompanyDBmsgResponse extends org.ros.internal.message.Message{
	
	// Field descriptor #5 Ljava/lang/String;
		  public static final java.lang.String _TYPE = "com/questit/accompany/msgs/AccompanyDBmsgResponse";
		  
		  // Field descriptor #5 Ljava/lang/String;
		  public static final java.lang.String _DEFINITION = "int64 code\nint64 sonRes\nstring answer\n";
		  
		  // Method descriptor #11 ()J
		  public abstract String getAnswer();
		  
		  // Method descriptor #13 (J)V
		  public abstract void setAnswer(String a);
		  
		  // Method descriptor #11 ()J
		  public abstract long getCode();
		  
		  // Method descriptor #13 (J)V
		  public abstract void setCode(long a);
		  
		  // Method descriptor #11 ()J
		  public abstract long getSonRes();
		  
		  // Method descriptor #13 (J)V
		  public abstract void setSonRes(long a);

}

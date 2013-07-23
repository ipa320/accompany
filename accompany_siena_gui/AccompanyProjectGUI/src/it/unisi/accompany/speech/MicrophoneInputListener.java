package it.unisi.accompany.speech;


public interface MicrophoneInputListener {
  public void processAudioFrame(short[] audioFrame);
}

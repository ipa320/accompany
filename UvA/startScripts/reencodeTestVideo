
cd ../testData

if [ ! -f testVideo.flv ];
then
  echo "===================================================="
  echo "reencoding wcam_20120112_vid4.avi to testVideo.flv as"
  echo "it uses a format that gstreamer can't read"
  echo "===================================================="
  mencoder -nosound wcam_20120112_vid4.avi -o testVideo.flv -ovc lavc
fi

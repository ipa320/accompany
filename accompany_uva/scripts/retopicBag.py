import rosbag
import sys

if len(sys.argv)<4:
    print "usage: "+sys.argv[0]+" <input-bag> <output-bag> <topic-prefix>"
    sys.exit(0)

inputBag=sys.argv[1]
outputBag=sys.argv[2]
topicPrefix=sys.argv[3]

with rosbag.Bag(outputBag, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(inputBag).read_messages():
        outbag.write(topicPrefix + topic, msg, t)

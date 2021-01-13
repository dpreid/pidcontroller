#! /bin/sh

#Script for initialising VARIABLE GOVERNOR remote lab.
#vw and ffmpeg automatically run

#./vw stream

#ffmpeg -f v4l2 -framerate 25 -video_size 640x480 -i /dev/video0 -f mpegts -codec:v mpeg1video -s 640x480 -b:v 1000k -bf 0 http://localhost:8888/ts/video0

echo "starting VARIABLE GOVERNOR"

curl -X POST -H "Content-Type: application/json" -d '{"stream":"video","destination":"wss://video.practable.io:443/in/dpr/video1","id":"0"}' http://localhost:8888/api/destinations

websocat tcp-listen:127.0.0.1:9999 ws://127.0.0.1:8888/ws/governor0 --text &

echo "waiting for websocat"

sleep 3

socat /dev/ttyACM0,echo=0,b57600,crnl tcp:127.0.0.1:9999 &

curl -X POST -H "Content-Type: application/json" -d '{"stream":"governor0","destination":"wss://video.practable.io:443/bi/dpr/governor0","id":"5"}' http://localhost:8888/api/destinations

echo "remote lab ready"

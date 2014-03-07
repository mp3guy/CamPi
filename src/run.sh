#!/bin/bash
for (( ; ; ))
do
    echo "Watching"
	./raspiyuv
	if [ $? -eq 1 ]
	then
		echo "Recording"
		file=$(date +"%F-%T")
		raspivid -o ~/$file.h264 -t 5000 -n
		echo "Finished Recording"
		echo -e "To: yourusername@gmail.com\nSubject: Motion detected\n\n$file.h264" > msg.txt
		ssmtp -s yourusername@gmail.com < msg.txt
		rm msg.txt
	fi
done

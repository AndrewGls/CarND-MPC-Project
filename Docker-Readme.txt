0. Run Xming if GUI will be used!
1. Create Dockerfile in ptoject folder.
2. in the `install-ubuntu.sh` you need to remove the `sudo`, because this doesn't run in docker
3. `docker build -t mpcp .` in your project directory
4. `docker run -p 127.0.0.1:4567:4567 mpcp ./mpc` (edited)
  or with GUI X11: "docker run -p 127.0.0.1:4567:4567 -e DISPLAY=192.168.0.26:0.0 -it mpcp ./mpc"
  where 192.168.0.26:0.0 is replaced by computer's IPv4!!!
	
5. start your sim and they will connect automatically
sim must run  in Windows

---------------------
Quizze mpc_to_line:
1. Run Xming if GUI will be used!
2. docker build -t mpcp .
3. docker run -p 127.0.0.1:4567:4567 -e DISPLAY=192.168.1.2:0.0 -it mpc
2. or "docker run -e DISPLAY=192.168.1.2:0.0 -it mpc ./build/mpc"


close:
docker ps
docker kill <name of container>
see below:
sorry... do  `docker ps`... then lookup the ID (first column) ... then `docker kill <first 3 chracters of the ID>`


open the sh-file in Notepad++ and then Edit>EOL Conversion > Unix/OSX Format ... then save

https://carnd.slack.com/messages/@jfjensen/files/F5DMCMHQT/
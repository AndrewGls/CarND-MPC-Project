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


=====================
Basic Build Instructions:
Clone this repo.
Make a build directory: mkdir build && cd build
Compile: cmake .. && make
Run it: ./mpc.




===========================
Param set2:
===========================
constexpr size_t N = 40/2/2;
constexpr double dt = 2*0.1;

constexpr double ref_v = 90;
// Weights to balance cte, epsi and distance to target speed, used during the cost error calculation.
constexpr double coeff_cte = 100.;
constexpr double coeff_epsi = 100.;
constexpr double coeff_v = 1.;
// Penalization coefficients:
constexpr double coeff_derivative_delta = 100.; // increase smoothness driving (smoothness steering)
constexpr double coeff_derivative_a = 500;      // increase smoothness of acceleration
constexpr double coeff_penalize_delta = 500.;   // minimizes the use of steering.
constexpr double coeff_penalize_a = 100.;       // minimizes the use of acceleration.


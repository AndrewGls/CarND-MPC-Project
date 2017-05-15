1. Create Dockerfile in ptoject folder.
2. in the `install-ubuntu.sh` you need to remove the `sudo`, because this doesn't run in docker
3. `docker build -t mpcp .` in your project directory
4. `docker run -p 127.0.0.1:4567:4567 mpcp ./mpc` (edited)
5. start your sim and they will connect automatically
sim must run  in Windows



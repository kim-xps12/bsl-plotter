# Use terminator instead of tmux
```bash
# Outside the docker container
cd bsl_plotter
sudo apt update
sudo apt install terminator
# If using customized settings
mkdir -p ~/.config/terminator/
cp terminator/config ~/.config/terminator/config
# Launch terminator(Run after `docker-compose up -d`)
cd docker_ros/
docker-compose up -d
cd ..
terminator
```
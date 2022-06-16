# Waypoint Node
## Build the docker image
```
git clone https://github.com/raryant/Waypoint
cd Waypoint
docker build --tag foxy-waypoint .
```
## running docker
```
docker run -it foxy-waypoint
```
## if you run the program in windows, use VcXSrv with provided configuration file

## run the program 
### if using dwb controller
#### terminal 1
```
docker run -it foxy-waypoint
./launch_type1.sh
```
#### terminal 2
```
docker run -it foxy-waypoint
./launch_waypoint.sh
```
### if using dwb controller
#### terminal 1
```
docker run -it foxy-waypoint
./launch_type2.sh
```
#### terminal 2
```
docker run -it foxy-waypoint
./launch_waypoint.sh
```
## run the unit test of waypoint
```
docker run -it foxy-waypoint
python3 /mnt/test_waypoint.py
```
## for modifying the point
change the points.json
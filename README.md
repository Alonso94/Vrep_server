#Vrep_server

To run the expirement on the server:
```bash
sudo docker pull 3liyounes/vrep_rozum
sudo docker run -it -p 172.17.0.1:19999:19999/tcp -p 8888:8888\
           --name vrep_rozum 3liyounes/vrep_rozum
```
Copy the url given by jupyter on the command line, and change:
```
127.0.0.1 -> <remote ip address>
```



 Build the docker file by:
```bash
docker build -t 3liyounes/vrep_rozum .
docker run -it --name vrep_rozum vrep_rozum:1.0 bash
```
After the first time, you can just use the container:
```bash
docker container start vrep_rozum
docker exec -it vrep_rozum
```
Inside the container run the file:
```bash
cd V-REP_PRO_EDU_V3_5_0_Linux
xvfb-run ./vrep.sh -h &
```
After have this working; you can use env_sim after running docker with the following parameters:
```bash
docker run -it -p 172.17.0.1:19999:19999/tcp -p 8888:8888\
           --name vrep_rozum 3liyounes/vrep_rozum
```
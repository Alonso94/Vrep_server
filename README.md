#Vrep_server
 Build the docker file by:
```bash
docker build -t vrep_rozum:1.0
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
xvfb-run ./vrep.sh -h
```
#Vrep_server

To run the expirement on the server:
```bash
sudo docker pull 3liyounes/vrep_rozum
sudo docker run -it -p 172.17.0.1:19999:19999/tcp -p 8888:8888 -p 6006:6006\
           --name vrep_rozum 3liyounes/vrep_rozum
```
Copy the url given by jupyter on the command line, and change:
```
127.0.0.1 -> <remote ip address>
```

After the first time:
```bash
sudo docker container start -i vrep_rozum
```
To kill jupyter after finishing the training
```bash
lsof -ti:8086 | xargs kill
```
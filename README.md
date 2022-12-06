# Swarm Robotics Projects
I built three decentralized robot controllers as a part of Northwestern University’s Swarms and Multi-Robot Systems ([MECH_ENG 409](https://www.mccormick.northwestern.edu/mechanical/academics/courses/descriptions/409-swarms-and-multi-robot-systems.html)) course to learn more about the state of the art research that addresses the challenges of controlling large groups of robots. See my [portfolio](https://billyen33.com/swarm.html) for a detailed description of these three projects.

## Instructions to Run My Code
1. First install [Docker Desktop](https://www.docker.com/)
2. Clone my GitHub repository to your preferred directory
```
git clone https://github.com/billyen33/Swarm_Robotics_Projects.git
```
3. Open up two command prompt windows on your computer (make sure you run them as administrator)
4. On the first command prompt window, go to the directory where you saved the project you want to run (the localization project, for example)
```
cd MyFolder/Swarm_Robotics_Project/localization
```
5. Go to the start.txt file in the project you want to run and copy the command there, paste it into the terminal, and press enter
6. Open Docker Desktop, go to Container, then click on the "Open in Browser" browser in the side pop-up menu
7. The Docker simulation should show up on your browser as a new window in a few seconds, enjoy!
8. To stop the simulation (or to rerun it in case of a connection error), go to the second command prompt and enter in the following command:
```
docker rm -f cs-local
```

## References
- [Shape Formation in Homogeneous Swarms Using Local Task Swapping](https://ieeexplore.ieee.org/document/9000788)
- [Organizing a Global Coordinate System from Local Information on an Ad Hoc Sensor Network](https://www.researchgate.net/publication/221284158_Organizing_a_Global_Coordinate_System_from_Local_Information_on_an_Ad_Hoc_Sensor_Network)
- [Segregation in swarms of mobile robots based on the Brazil nut effect](https://ieeexplore.ieee.org/document/5353942)
- [Reynolds flocking in reality with fixed-wing robots: communication range vs. maximum turning rate](https://ieeexplore.ieee.org/document/6095129)

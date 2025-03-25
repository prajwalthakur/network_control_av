# Network Controlled Autonomous Mobile Robot

# System Architecture Diagram

```mermaid
flowchart TB
    TCPServer["**Server**"]
    RobotClient["**Robot Client**"]
    MotionPlanner["Motion Planner"]
    RobotDynamics["Robot Dynamics"]
    
    TCPServer <--> RobotClient
    MotionPlanner --> TCPServer
    RobotClient --> RobotDynamics 
    
```

https://github.com/user-attachments/assets/31855e9c-6aaa-4744-acb6-ca8be0d7e39e



## sfml dependency
apt-get install libsfml-dev

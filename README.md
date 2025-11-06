## <div align = "center">Robot Navigation in Unknown and Cluttered Workspace with Dynamical System Modulation in Starshaped Roadmap</div>

<div align="left">
<a href="https://ieeexplore.ieee.org/document/11128318"><img src="https://img.shields.io/badge/Paper-IEEE ICRA-004088.svg"/></a>
<a href="https://arxiv.org/abs/2403.11484"><img src="https://img.shields.io/badge/ArXiv-2403.11484-da282a.svg"/></a>

</div>


<div align="left">

  <video src="https://private-user-images.githubusercontent.com/40656775/369309215-a8d93bef-98d8-499c-b7d2-f3f0b796a9f6.mp4?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3MjY4MTc4MzIsIm5iZiI6MTcyNjgxNzUzMiwicGF0aCI6Ii80MDY1Njc3NS8zNjkzMDkyMTUtYThkOTNiZWYtOThkOC00OTljLWI3ZDItZjNmMGI3OTZhOWY2Lm1wND9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNDA5MjAlMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjQwOTIwVDA3MzIxMlomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTNmNjM1Y2MyY2ViNGZlNmU5NGJmMDkzYjdjMDM3NjVmMzhhMTYwNjVjYzYxMzNiOTNlODAxYTk1ODIxOTgyYWQmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.-ztOee5J4n66FPi5SZn4NSIuc9hndd_utxmNtJM1Vi4"
    width="400" height="225" controls loop muted playsinline style="border-radius: 10px; box-shadow: 0 2px 8px #aaa;">
    Your browser does not support the video tag.
  </video>
</div>

## Compile
```bash
# prerequest
sudo apt install ros-noetic-turtlebot3-description ros-noetic-fake-localization ros-noetic-jsk-rviz-plugins

mkdir -p sr_ws/src
cd sr_ws/src
git clone https://github.com/kkkkkaiai/starshaped_roadmap.git
cd .. && catkin_make
```
## Execute

```bash
# launch the environment
roslaunch starworlds turtlebot_simulation.launch

# run the node (in the ss_rm directory)
python star_ros.py
```



## Citing
```
@INPROCEEDINGS{11128318,
  author={Chen, Kai and Liu, Haichao and Li, Yulin and Duan, Jianghua and Zhu, Lei and Ma, Jun},
  booktitle={2025 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={Robot Navigation in Unknown and Cluttered Workspace with Dynamical System Modulation in Starshaped Roadmap}, 
  year={2025},
  pages={10140-10146},
  keywords={Navigation;Heuristic algorithms;Source coding;Modulation;Aerospace electronics;Robot sensing systems;Real-time systems;Stability analysis;Planning;Dynamical systems},
  doi={10.1109/ICRA55743.2025.11128318}
}
```

## Thanks

- [Dynamic Obstacle Avoidance](https://github.com/hubernikus/dynamic_obstacle_avoidance)
- [Fast Obstacle Avoidance](https://github.com/hubernikus/fast_obstacle_avoidance)
- [Various Tools](https://github.com/hubernikus/various_tools)

## <div align = "center">Robot Navigation in Unknown and Cluttered Workspace with Dynamical System Modulation in Starshaped Roadmap</div>

<div align="center">

<a href="https://arxiv.org/abs/2403.11484"><img src="https://img.shields.io/badge/ArXiv-2403.11484-da282a.svg"/></a>

</div>

## Abstract

Compared to conventional decomposition methods that use ellipses or polygons to represent free space, starshaped representation can better capture the natural distribution of sensor data, thereby exploiting a larger portion of traversable space. This paper introduces a novel motion planning and control framework for navigating robots in unknown and cluttered environments using a dynamically constructed starshaped roadmap. Our approach generates a starshaped representation of the surrounding free space from real-time sensor data using piece-wise polynomials. Additionally, an incremental roadmap maintaining the connectivity information is constructed, and a searching algorithm efficiently selects short-term goals on this roadmap. Importantly, this framework addresses deadend situations with a graph updating mechanism.
To ensure safe and efficient movement within the starshaped roadmap, we propose a reactive controller based on Dynamic System Modulation (DSM). This controller facilitates smooth motion within starshaped regions and their intersections, avoiding conservative and short-sighted behaviors and allowing the system to handle intricate obstacle configuration s in unknown and cluttered environments.
Comprehensive evaluations in both simulations and real-world experiments show that the proposed method achieves higher success rates and reduced travel times compared to other methods. It effectively manages intricate obstacle configurations, avoiding conservative and myopic behaviors. 

https://private-user-images.githubusercontent.com/40656775/369291665-7890182b-8b99-4edf-8640-3447e00a1b34.mp4?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3MjY4MTUwOTksIm5iZiI6MTcyNjgxNDc5OSwicGF0aCI6Ii80MDY1Njc3NS8zNjkyOTE2NjUtNzg5MDE4MmItOGI5OS00ZWRmLTg2NDAtMzQ0N2UwMGExYjM0Lm1wND9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNDA5MjAlMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjQwOTIwVDA2NDYzOVomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTVhMDExZTEwNjBjM2EyOWU3MDg1MDQ2NGRkMDM3OTM4MjdiOGQxOTU0YzA5ODdiMzVlZjM3OTJhZDYxN2UyMDkmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.s7rYouSisgHKfx_fn13Lw_xuFTjMYBLa9pLNbkQKJkA

## Citing
```
@misc{chen2024robotnavigationunknowncluttered,
      title={Robot Navigation in Unknown and Cluttered Workspace with Dynamical System Modulation in Starshaped Roadmap}, 
      author={Kai Chen and Haichao Liu and Yulin Li and Jianghua Duan and Lei Zhu and Jun Ma},
      year={2024},
      eprint={2403.11484},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2403.11484}, 
}
```

---
title: 'DSO++: Modern reimplementation of Direct Sparse Odometry'
tags:
  - SLAM
  - odometry
authors:
  - name: Sergei Solonets^[co-first author]
    orcid: 0000-0002-3469-7489
    affiliation: "1, 2" 
  - name: Igor Ilin^[co-first author]
    affiliation: 1
    orcid: 0000-0002-1834-4894
  - name: Daniil Sinitsyn^[co-first author]
    affiliation: 1
    orcid: 0000-0002-9102-9065
  - name: Maksim Smolskii
    affiliation: 1
    orcid: 0000-0001-9414-0853
  - name: Oleg Chemokos 
    affiliation: 1
    orcid: 0000-0003-2830-4340
  - name: Eugene Nikolsky
    affiliation: 1
affiliations:
 - name: Roadly Inc, USA
   index: 1
 - name: Technical University of Munich, Germany
   index: 2
date: 17 May 2022
bibliography: paper.bib

---


# Summary

We present DSO++, a modern reimplementation of one of the most well-known Simultaneous Localization And Mapping (SLAM) engines: Direct Sparse Odometry (DSO) [@engel2017direct]. Similar to its ancestor, our system produces a trajectory and a sparse point cloud from a video sequence. It acts on original image data and directly estimates motion by minimizing the photometric error. Compared to the original code, our version is covered with tests and is modular. We also designed the code with the possibility of being easily extended especially concentrating on the proposed extensions (various camera model formulations, multi-sensor setup, etc.). Another contribution is extensive testing of the functionality.

# Statement of need

Direct simultaneous localization and mapping is an active research topic in the computer vision community. One of the first working examples of direct SLAM is DSO. Since the release of the DSO code, researchers have proposed several extensions to the original method, introducing such features as rolling shutter motion modeling [@schubert2018direct], stereo [@wang2017stereo], deep learning aided depth inference [@yang2020d3vo] [@gnnet] and other [@9669044] [@gao2018ldso] [@omni]. Almost all of the extensions are released by the same computer vision lab, most likely due to the complexity of the original DSO code. The code was not designed with a presumption of continuous development and, therefore, is hard to maintain and extend, especially by outside researchers from other groups. 

The research purpose of this software is twofold. Firstly, it provides researchers with implemented and tested modules for direct SLAM: photometric bundle adjustment, feature metric bundle adjustment, marginalization, etc., which could be useful for any subsequent work as well as student projects. Secondly, similar to any other Structure From Motion algorithms it can be directly used by researchers in the area of photogrammetry, 3D reconstruction, differentiable rendering, and other similar areas. In those areas, researchers need estimated camera motion and/or approximate scene structure for further enhancement. Today it is common to use camera poses obtained from the indirect SLAM algorithms, which are shown to be less accurate compared to direct ones [@engel2017direct].  

# Contribution

Compared to the original implementation we propose following improvements:

- **Modularity**. Original DSO consists of a few source files with extensive code duplication. For example, the code responsible for ray reprojection can be found in 30-50 different places. It not only makes the development process error prone but also makes the code extremely hard to modify. However, for some followup papers, this chunk of code was the only change (rolling shutter DSO[@schubert2018direct], omnidirectional DSO[@omni]). In our implementation, code is organized in a way that those kinds of contributions would require only a single modification. 

- **Tests**. Every module in our implementation is covered with a test. Tests not only insure the validity of the code but also declare the essence of this module and show their usage. There are no tests in the original DSO.

- **Extensibility**. Functionality of every follow-up research have been or can be developed within our framework. It can be turned on or off dynamically based on the configuration file. It prevents duplication of code among several repositories which insures every contribution affects previous and following contributions. 





# References

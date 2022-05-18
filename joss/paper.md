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

We present DSO++, a modern reimplementation of one of the most well-known simultaneous localization and mapping (SLAM) engines. As well as its ancestor our system produces a trajectory and a sparse point cloud from a video sequence. It acts on original image and directly estimate motion by minimizing the photometric error. Compared to the original code, our version is covered with tests and modular. We also designed the code with the possibility of being extended especially concentrating on the examples of the follow-up papers. 

# Statement of need

Direct simultaneous localization and mapping (SLAM) is an active research topic in the computer vision community. One of the first working examples of direct SLAM is Direct Sparse Odometry(DSO) by Engel et al [@engel2017direct]. Since the release of the DSO code, researchers have proposed several extensions to the original method [@schubert2018direct] [@wang2017stereo] [@gao2018ldso] [@9669044] [@yang2020d3vo]. The original DSO code was not designed with a presumption of continuous development and, therefore, does not follow many good software principles. Consequently, follow-up works inherit the mistakes in the code as well as poor software practices. To facilitate the research in the field we propose our reimplementation of the DSO designed to be easily modifiable and extendable. Another contribution is extensive testing of the functionality and supporting mathematical hypotheses.

# References

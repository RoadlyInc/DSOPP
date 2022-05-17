---
title: 'DSO++: Reimplementation of Direct Sparse Odometry'
tags:
  - SLAM
  - odometry
  - direct
  - photometric bundle adjustment
authors:
  - name: Sergei Solonets^[co-first author]
    orcid: 0000-0002-3469-7489
    affiliation: "1, 2" 
  - name: Igor Ilyin^[co-first author]
    affiliation: 1
  - name: Daniil Sinitsyn^[co-first author]
    affiliation: 1
    orcid: 0000-0002-9102-9065
  - name: Maxim Smolsky
    affiliation: 1
  - name: Oleg Chemokos 
    affiliation: 1
  - name: Eugene Nikolsky
    affiliation: 1
affiliations:
 - name: Roadly Inc
   index: 1
 - name: Technical University of Munich
   index: 2
date: 17 May 2022
codeRepository: https://github.com/RoadlyInc/DSOPP
license: LGPLv3
bibliography: paper.bib

---

# Summary

Direct simultaneous localization and mapping (SLAM) is an active research topic in the computer vision community. One of the first working examples of direct SLAM is Direct Sparse Odometry(DSO) by Engel et al [@engel2017direct]. Since the release of the DSO code, researchers have proposed several extensions to the original method [@schubert2018direct] [@wang2017stereo]  [@gao2018ldso] [@9669044] [@yang2020d3vo]. The original DSO code was not designed with a presumption of continuous development and, therefore, does not follow many good software principles. Consequently, follow-up works inherit the mistakes in the code as well as poor software practices. To facilitate the research in the field we propose our reimplementation of the DSO designed to be easily modifiable and extendable. Another contribution is extensive testing of the functionality and supporting mathematical hypotheses.   

# References

<p align="center">
	<img src="assets/logo.svg" alt="XR-UCalib" width="600"/>
</p>

<h2 align="center">
	A Unified Spatiotemporal Calibration Framework for Heterogeneous Multi-Sensor Systems in Extended Reality
</h2>

<p align="center">
	<img src="assets/calib_result.gif" alt="Calibration result demo" width="100%"/>
</p>

> **Status:** The paper associated with this project is currently under review. The full source code, documentation, and datasets will be released upon acceptance.

## 📖 Overview

XR-UCalib is a unified spatiotemporal calibration framework designed for immersive XR systems, delivering high-precision calibration to meet their stringent requirements. Key features include:

- **Unified Calibration:**
	Provides a unified framework for calibrating typical heterogeneous sensors in XR systems, avoiding the error accumulation of sequential processing. Covers global- and rolling-shutter cameras, IMUs, and magnetometers, supports arbitrary sensor counts, and is readily extensible to additional modalities.

- **Joint Spatiotemporal Estimation:**
	Simultaneous estimation of extrinsic parameters (rotation and translation) and temporal offsets within a single continuous-time optimization framework.

- **Flexible Low-Overlap Calibration:**
	Supports arbitrary numbers of calibration targets (e.g., AprilTag boards), enabling robust calibration for low-overlap and non-overlap camera rigs.

System flowchart is shown below.

![System flowchart](assets/system_flawchart.png)

## 📚 Reference

```bibtex
@article{shu2026xrucalib,
  title   = {XR-UCalib: A Unified Spatiotemporal Calibration Framework
             for Heterogeneous Multi-Sensor XR Systems},
  author  = {Zichao Shu and Lijun Li and Xudong Zhang and Hongjun Fang and
             Xinlei Chu and Zetao Chen and Jianyu Wang},
  journal = {Under Review},
  year    = {2026},
}
```

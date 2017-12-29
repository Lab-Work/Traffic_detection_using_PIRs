# Traffic_detection_using_PIRs
Yanning Li, Christopher Chen, Fangyu Wu, December, 2017

## 1) Overview
This repository contains the source code developed for traffic detection and vehicle speed estimation using PIR sensors. This results are reported in a journal paper *"Traffic detection via energy efficient passive infrared sensing"* by Yanning Li, Christopher Chen, Fangyu Wu, Christian Claudel, and Daniel Work, which was submitted to *IEEE Transactions on Intelligent Transportation Systems*.

## 2) License

This software is licensed under the *University of Illinois/NCSA Open Source License*:

**Copyright (c) 2017 The Board of Trustees of the University of Illinois. All rights reserved**

**Developed by: Department of Civil and Environmental Engineering University of Illinois at Urbana-Champaign**

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal with the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimers. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimers in the documentation and/or other materials provided with the distribution. Neither the names of the Department of Civil and Environmental Engineering, the University of Illinois at Urbana-Champaign, nor the names of its contributors may be used to endorse or promote products derived from this Software without specific prior written permission.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS WITH THE SOFTWARE.

## 3) Structure

This repository is devided to three folders/repositories, i.e., `/Detection platform/`, `ITS2017_Validation`, and `Traffic_Sensors_Algorithms`, primarily developed by Christopher Chen, Fangyu Wu, and Yanning Li, respectively.  


- `Detection platform`: the source code for the hardware traffic sensing nodes, i.e., firmware and sensor libraries. 

- `ITS2017_Validation`: the repository for generating the true traffic detection and speed using videos and computer vision. The true detection and speeds are used for evaluating the performance of the energy efficient PIR sensors for traffic detection.

- `Traffic_Sensors_Algorithms`: the repository that contains the source code for processing the PIR sensor data for traffic detection and speed estimation. 
	
Please refer to the readme file in each folder or repository for how to use the code. 







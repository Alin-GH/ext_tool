# visualization of point cloud and labeled informations 

## Introduction
seleted fold contains point cloud infos as pcd files and related images json_files are given labeled files 
to prepare training data as kitti-like format use json2ttxt.py to extract useful information json_out as the output 

## trunk-like 
The label files contain the following information, which can be read and

#Values    Name      Description
----------------------------------------------------------------------------
   1    name         Describes the type of object: 'Car', 'Bus', 'Truck',
   
   3    location     3D object location x,y,z in object coordinates (in meters)

   3    dimensions   3D object dimensions: height, width, length (in meters)

   1    alpha        Observation angle of object, ranging [-pi..pi]
   
   4    bbox         2D bounding box of object in the image (0-based index):
                     contains left, top, right, bottom pixel coordinates




## Requirements
All codes are tested under the following environment:
*   Ubuntu
*   Python 3.7
*   opencv
*   numpy


## run 
1. use `conda` to manage the environment:
```
conda create -n smoke python=3.7
```

2. activate environment:
```
conda activate smoke
```

3. extract infomations:
```
python prex.py
```

4. show result:
```
./view.sh
```
or 
```
python showresult.py
```

## Acknowledgement

[KITTI 3D Object Dataset](http://www.cvlibs.net/datasets/kitti/eval_object.php?obj_benchmark=3d)

[SMOKE: Single-Stage Monocular 3D Object Detection via Keypoint Estimation](https://arxiv.org/pdf/2002.10111.pdf)


thanks.
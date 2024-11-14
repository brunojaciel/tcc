
Bottles and Cups Detection - v24 2023-12-03 3:06pm
==============================

This dataset was exported via roboflow.com on December 3, 2023 at 8:10 AM GMT

Roboflow is an end-to-end computer vision platform that helps you
* collaborate with your team on computer vision projects
* collect & organize images
* understand and search unstructured image data
* annotate, and create datasets
* export, train, and deploy computer vision models
* use active learning to improve your dataset over time

For state of the art Computer Vision training notebooks you can use with this dataset,
visit https://github.com/roboflow/notebooks

To find over 100k other datasets and pre-trained models, visit https://universe.roboflow.com

The dataset includes 12900 images.
Bottles, Cups are annotated in YOLOv8 format.

The following pre-processing was applied to each image:
* Auto-orientation of pixel data (with EXIF-orientation stripping)
* Resize to 640x640 (Stretch)

The following augmentation was applied to create 3 versions of each source image:
* 50% probability of horizontal flip
* Equal probability of one of the following 90-degree rotations: none, clockwise, counter-clockwise
* Random rotation of between -30 and +30 degrees
* Random brigthness adjustment of between -25 and +25 percent
* Random Gaussian blur of between 0 and 2 pixels
* Salt and pepper noise was applied to 5 percent of pixels



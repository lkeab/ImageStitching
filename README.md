#  ImageStitchig
Panoramic multi-layer image stitching based on opencv 2.49 (opencv only provides single-layer stitching)
QT+VS2013 joint development, QT used as interface design
Import multiple images in layers, and finally output a whole panoramic image 

# Main process:
1. Input source image and program parameters
2. Feature point detection, to determine whether to use surf or orb, the default is surf
3. Matching the feature points of the image, using the nearest neighbor and second nearest neighbor methods, and save the confidence of the two optimal matches.
4. Sort the images and save the images with high confidence in the same set, delete the matches between the images with lower confidence, and get the image sequence that can be matched correctly. In this way, all matches with confidence higher than the threshold are merged into one set.
5. Roughly estimate the camera parameters for all images, and then calculate the rotation matrix
6. Use the beam averaging method to further accurately estimate the rotation matrix.
7. Waveform correction, horizontal or vertical
8. Splicing 
9. Fusion, multi-band fusion, illumination compensation

Wuhan University Innovation and Entrepreneurship Project 2017.4

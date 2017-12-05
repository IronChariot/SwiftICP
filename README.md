# SwiftICP
Simple ICP Algorithm implemented in Swift with as few dependencies as possible

Uses KDTree (https://github.com/Bersaelor/KDTree).

Include an onscreen point cloud and its transformation using SceneKit. The example point cloud does not get great results, but this has been verified to be the case with ICP in CloudCompare with the same example. Currently does not include any way of dealing with outliers and partial overlap, hence the inadequate result with this example.

Scanner branch removes rotation from ICP result and limits the point pairs to thus with Z-axis separation beyond a certain threshold. This correctly deals with the example, and should be useful when using a 3D scanner to scan a surface from several angles - the depth (z-axis) is the most important thing to match up, and the rotation of the scanner itself is well recorded and compensated for before ICP is used, thus only translation is necessary to finish the registration of the different frames.

Based on code and information from:

http://nghiaho.com/?page_id=671 (Finding optimal rotation/translation between two sets of points)

https://github.com/abreheret/icp-opencv/blob/master/src/icp-opencv/icp.c (ICP implementation using OpenCV)

https://gist.github.com/wannyk/04c1f48161780322c0bb (MATLAB SVD in Swift)

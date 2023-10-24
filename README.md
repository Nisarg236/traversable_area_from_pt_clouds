<h1> Using the pcdWindows package </h1>
<h3> Point Cloud after removal of points > certain height.
![Image](https://github.com/Nisarg236/traversible_area_from_pt_clouds/assets/71684502/460162bc-bab1-4a86-b34c-80bb64248d10)

<h3> Raw elevation map
![image](https://github.com/Nisarg236/traversible_area_from_pt_clouds/assets/71684502/01f64e18-f6a7-4fea-b109-e92ec0c4bcdd)

<h3> Elevation map after interpolation
![image](https://github.com/Nisarg236/traversible_area_from_pt_clouds/assets/71684502/be325bef-5fc3-46fe-98de-cbae70906177)

<h3> Traversability
![image](https://github.com/Nisarg236/traversible_area_from_pt_clouds/assets/71684502/f7bae5a4-f3f2-4ee3-8823-5c8b76db7fdf)


<h3>This package is used to convert a point cloud to a 2D map, it runs in two steps, first it takes in the point cloud and then loads the poses. After this it divides it into small parts, removes points higher than the robot height and then applies difference of normals segmentation on each of the window and saves the obstacle points to a separate point cloud. Then it creates an empty gridmap and adds an obstacle layer and then projects the obstacle points on the 2d grid. Here majority of the obstacles are marked like trees, walls, poles, etc. But curbs are not detected. After this it converts the point cloud to elevation map -> fills empty spaces and then calculates the traversibility map where each cell is assigned value by how much slope it makes to its neighboring cells, in this the curbs are visible. 

To run the code, edit the path to the input point cloud and also the poses, build it and then
```
rosrun pcdWindows pcdWindows
```

It will take time some time to run, close other programs to save RAM. After it is done, the 2D map created by detecting obstacles from DoN can viewed under the topic /obstacles. Save it using:
```
rosrun map_server map_saver map:=/obstacles
```
And it will save two the elevation map and the traversability map as images. Now crop a small portion of it which contains a curb and save. Give the path to the cropped area in the hsv_trackbar.py script and then adjust the sliders to only segment the curbs and note the value of the sliders.

Then in color_segment.py, insert the noted values and the path to the whole traversability map and then run it. This will save the image which contains the sidewalks. After this use the merge_2.py script to merge the curbs with the obstacles map. This will give a final occupacy grid. which contains curbs, and other static obstacles such as trees, poles, walls, etc. </h3>

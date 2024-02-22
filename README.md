<h1> Using the package </h1>

<h3> Point Cloud after removal of points > certain height.
  
![cloud](https://github.com/Nisarg236/traversible_area_from_pt_clouds/assets/71684502/124882d1-3cef-4c69-bdaa-e8ba57909564)


<h3> Raw elevation map
  
![raw_elevation_map](https://github.com/Nisarg236/traversible_area_from_pt_clouds/assets/71684502/420b4686-dfcf-43f5-96f5-74df0f05e027)

<h3> Elevation map after interpolation
  
![elevation_map_interpolated](https://github.com/Nisarg236/traversible_area_from_pt_clouds/assets/71684502/de50edb6-bd04-4d38-9943-b8cd31735b9c)

<h3> Traversability
  
![traversability](https://github.com/Nisarg236/traversible_area_from_pt_clouds/assets/71684502/d91b4dbe-d000-45d5-b5a7-1d6671355974)

<h3> Calculation of Traversability from Elevation matrix

![image](https://github.com/Nisarg236/traversable_area_from_pt_clouds/assets/71684502/2f340a03-19e2-487a-8023-2458783511f4)


<h3>This package is used to convert a point cloud into a 2D map, it runs in two steps, first it takes in the point cloud and then loads the poses. After this it divides it into small parts, removes points higher than the robot height and then applies difference of normals segmentation on each of the window and saves the obstacle points to a separate point cloud. Then it creates an empty gridmap and adds an obstacle layer and then projects the obstacle points on the 2d grid. Here majority of the obstacles are marked like trees, walls, poles, etc. But curbs are not detected. After this it converts the point cloud to elevation map -> fills empty spaces and then calculates the traversibility map where each cell is assigned value by how much slope it makes to its neighboring cells, in this the curbs are visible. 

To run the code, edit the path to the input point cloud and also the poses, build it and then
```
rosrun pcdWindows pcdWindows
```

It will take time some time to run, close other programs to save RAM. After it is done, the 2D map created by detecting obstacles from DoN can viewed under the topic /obstacles. Save it using:
```
rosrun map_server map_saver map:=/obstacles

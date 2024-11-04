# cleaningbot_navigation_sim
description  
demo img  

# 1. Build & Run
dependencies cmd
```
sudo apt install clang-format-9
sudo apt install nlohmann-json3-dev
sudo apt install libeigen3-dev
```
build cmd  
```
colcon build --symlink-install 
```
run cmd  
```
ros2 run cleaningbot_navigation_sim cleaningbot_sim 
ros2 service call /load_plan_json cleaningbot_navigation_sim/srv/LoadPlanJson "{"plan_json": "/home/shanlee/ros2_ws/src/cleaningbot_navigation_sim/data/short.json"}"
```
debug cmd
```
bash format.bash
// test
```

# 2. Mathematic Notes
## Curvature
  - curvature definition: `k = 1/R` [1]
    - ![img](https://github.com/user-attachments/assets/26aaa8cc-257e-4ff6-a8b8-8c141269a21f)
  - three-point curvature approximation `k ≈ 4S/abc` or `k ≈ 2θ/(a+b)` [2][3]
    - ![img](https://github.com/user-attachments/assets/b76ed77a-2477-4ee0-b0e9-a3734f7a0c11)
    - ![img](https://github.com/user-attachments/assets/0680bda3-0907-434f-ba11-c107d2d3b506)
  - so that given `[x0,y0], [x1,y1], [x2, y2]` compute the `k1`:
    - `vecA = [x1-x0, y1-y0]`
    - `vecB = [x2-x1, y2-y1]`
    - `cos(θ) = vecA•vecB/|vecA||vecB|`
    - `avg_len = (|vecA|+|vecB|)/2 = (a+b)/2`
    - `k1 ≈ θ/avg_len`
    - `k1 ≈ 2* arccos(vecA•vecB/|vecA||vecB|) / (|vecA|+|vecB|) `
## Length of the path/trajectory/arc
  - `L ≈ sum(|veci|), i=[a,b]` [4]
    - ![img](https://github.com/user-attachments/assets/cfa7883c-0693-4996-9b1c-ec68789529ec)
## Area of the path/trajectory/arc
  - `A = L*W`, W is the footprint width `W = sqrt((x0-x1)^2+(x1-y1)^2)`, while this does not consider about the overlapping area
## Duration of traversing the path/trajectory/arc
  - `dt = dL / dv`
  - `t = sum(|veci|/vi), i=[a,b]`
## Interpolation
  - ![img](https://upload.wikimedia.org/wikipedia/commons/thumb/9/90/Comparison_of_1D_and_2D_interpolation.svg/375px-Comparison_of_1D_and_2D_interpolation.svg.png) [5]
  - cublic spline interpolation [6]
  - ![image](https://github.com/user-attachments/assets/92dc1118-157d-4641-ad05-e6f7bff5e9b5)
## Flood-Fill algo
  - ![image](https://github.com/user-attachments/assets/c84cce9a-05fe-44c6-83fc-1fb4a25c3fbb) [7]
  - ![img](https://upload.wikimedia.org/wikipedia/commons/7/7e/Recursive_Flood_Fill_4_%28aka%29.gif)
## DDA Raycasting algo
  - DDA is a fast algorithm typically used on square grids to find which squares a line hits (for example to draw a line on a screen, which is a grid of square pixels). [8]
  - ![image](https://github.com/user-attachments/assets/c4a7636c-8a9f-427d-a34a-70a50c41db60)
## arctan2
## line and the cross product

# 3. Ref
- [1] https://en.wikipedia.org/wiki/Curvature
- [2] https://math.stackexchange.com/questions/2445037/numerical-approach-to-find-curvature
- [3] Belyaev, Alexander G. "A note on invariant three-point curvature approximations (singularity theory and differential equations)." 1111 (1999): 157-164.
- [4] https://tutorial.math.lamar.edu/classes/calcii/arclength.aspx
- [5] https://en.wikipedia.org/wiki/Bicubic_interpolation
- [6] https://en.wikipedia.org/wiki/Cubic_Hermite_spline
- [7] https://en.wikipedia.org/wiki/Flood_fill
- [8] https://lodev.org/cgtutor/raycasting.html
